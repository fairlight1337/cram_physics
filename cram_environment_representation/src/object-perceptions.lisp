;;; Copyright (c) 2012, Lorenz Moesenlechner <moesenle@in.tum.de>
;;; All rights reserved.
;;; 
;;; Redistribution and use in source and binary forms, with or without
;;; modification, are permitted provided that the following conditions are met:
;;; 
;;;     * Redistributions of source code must retain the above copyright
;;;       notice, this list of conditions and the following disclaimer.
;;;     * Redistributions in binary form must reproduce the above copyright
;;;       notice, this list of conditions and the following disclaimer in the
;;;       documentation and/or other materials provided with the distribution.
;;;     * Neither the name of the Intelligent Autonomous Systems Group/
;;;       Technische Universitaet Muenchen nor the names of its contributors 
;;;       may be used to endorse or promote products derived from this software 
;;;       without specific prior written permission.
;;; 
;;; THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
;;; AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
;;; IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
;;; ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
;;; LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
;;; CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
;;; SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
;;; INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
;;; CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
;;; ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
;;; POSSIBILITY OF SUCH DAMAGE.

(in-package :cram-environment-representation)

(defvar *ignored-bullet-objects* nil)

(defclass perceived-object-data
    (desig:object-designator-data
     cram-manipulation-knowledge:object-shape-data-mixin)
  ((pose :reader pose :initarg :pose)
   (identifier :reader identifier :initarg :identifier)))

(defmethod clear-belief object-identifiers ()
  (clrhash *object-identifier-to-instance-mappings*))

(defgeneric register-object-designator-data (data &key type)
  (:documentation "Registers an object in the belief state. The object
is identified by its corresponding OBJECT-DESIGNATOR-DATA object. This
method is also responsible for entity resolution, i.e. for finding the
right object instance that needs to be updated and for adding new
instances if necessary.

The default implementation for the objects defined in
CRAM-MANIPULATION-KNOWLEDGE completely trust the object identifiers,
i.e. if an object instance for an identifier has been registered
already, the pose of the corresponding object in the world database is
just updated. Otherwise a new instance is created."))

(defgeneric object-mass (data)
  (:documentation "Returns the (maybe approximated) mass of an object
  identified by `data' which is of type OBJECT-DESIGNATOR-DATA."))

(defmethod register-object-designator-data
    ((data cram-manipulation-knowledge:object-shape-data-mixin) &key type)
  (declare (ignore type))
  nil)

(defmethod register-object-designator-data
    ((data cram-manipulation-knowledge:object-mesh-data-mixin) &key type)
  (let ((instance-name (or
                        (gethash (desig:object-identifier data)
                                 *object-identifier-to-instance-mappings*)
                        (setf (gethash (desig:object-identifier data)
                                       *object-identifier-to-instance-mappings*)
                              (gensym (string (desig:object-identifier data)))))))
    (prolog `(and (bullet-world ?world)
                  (-> (object ?world ,instance-name)
                      (assert ?world (btr:object-pose
                                      ,instance-name
                                      ,(desig:object-pose data)))
                      (assert ?world (btr:object btr:mesh ,instance-name
                                                 ,(desig:object-pose data)
                                                 :mesh ,(object-mesh data)
                                                 :mass ,(object-mass data)
                                                 :types ,(list type)
                                                 :disable-face-culling t)))))))

(defmethod register-object-designator-data
    ((data cram-manipulation-knowledge:object-point-data-mixin) &key type)
  (declare (ignore type))
  nil)

(defmethod object-mass ((data cram-manipulation-knowledge:object-mesh-data-mixin))
  (physics-utils:calculate-mass
   :mesh :points (cram-manipulation-knowledge:vertices data)))

(defun object-mesh (data)
  (declare (type cram-manipulation-knowledge:object-mesh-data-mixin data))
  (with-slots ((vertices cram-manipulation-knowledge:vertices)
               (faces cram-manipulation-knowledge:faces)) data
    (physics-utils:make-3d-model
     :vertices vertices
     :faces (physics-utils:fix-normals
             (map 'vector (lambda (indices)
                            (physics-utils:make-face
                             :points (mapcar (alexandria:curry #'elt vertices) indices)))
                  faces)
             :always-recalculate t))))

(defun ignore-bullet-object-during-perception (object-name)
  (setf *ignored-bullet-objects*
        (remove object-name *ignored-bullet-objects*))
  (push object-name *ignored-bullet-objects*))

(defun remove-disappeared-objects (object-names)
  "Removes objects from the current bullet world. They are refered to by the `name' property specified in their designator."
  (dolist (object-name object-names)
    (roslisp:ros-info (env rep) "Removing disappeared object `~a'" object-name)
    (crs:prolog
     `(and (btr:bullet-world ?w)
           (btr:retract
            (btr:object
             ?w ,object-name))))
    (plan-knowledge:on-event
     (make-instance 'plan-knowledge:object-removed-event
                    :object-name object-name))))

(defun add-appeared-objects (objects)
  "Adds objects to the current bullet world. In the world, they then consist of boxes of dimensions as specified in the `dimensions'
property in their designator."
  (dolist (object objects)
    (let ((pose (desig:desig-prop-value
                 (desig:desig-prop-value object 'desig-props::at)
                 'pose))
          (dimensions (or (desig:desig-prop-value
                           object 'desig-props:dimensions)
                          (vector 0.1 0.1 0.1)))
          (name (desig:desig-prop-value object 'desig-props:name)))
      ;; TODO(winkler): Fix me
      (crs:prolog `(and (btr:bullet-world ?w)
                        (btr:retract (btr:object ?w ,name))))
      (crs:prolog `(and (btr:bullet-world ?w)
                        (btr:assert
                         (btr:object
                          ?w btr:box ,name ,pose
                          :mass 0.1
                          :size ,(map 'list #'identity dimensions)))))
      ;; (crs:prolog
      ;;  `(and (bullet-world ?w)
      ;;        (assert
      ;;         (object
      ;;          ?w btr::mesh ,name ,pose
      ;;          :mass 0.1
      ;;          :mesh desig-props::mondamin :color (0.8 0.4 0.2)))))
      (plan-knowledge:on-event
       (make-instance 'plan-knowledge:object-perceived-event
                      :object-designator object
                      :perception-source 'generic)))))

(defun update-objects (objects)
  "Updates objects' poses in the current bullet world based on the
`name', and the `pose' properties in their respective designator."
  (dolist (object objects)
    (let ((pose (desig:desig-prop-value
                 (desig:desig-prop-value object 'at)
                 'pose))
          (name (desig:desig-prop-value object 'desig-props:name)))
      (crs:prolog
       `(and (btr:bullet-world ?w)
             (btr:assert
              (btr:object-pose
               ?w ,name ,pose)))))
    (plan-knowledge:on-event
     (make-instance 'plan-knowledge:object-updated-event
                    :object-designator object
                    :perception-source :generic))))

(defmethod cram-task-knowledge:objects-perceived (object-template object-designators)
  ;; Make sure that the current pose and everything is in the
  ;; bullet reasoning beliefstate.
  (plan-knowledge:on-event
   (make-instance 'plan-knowledge:robot-state-changed))
  (let* ((perceived-objects object-designators)
         (perceived-object-designators ;; Doesn't include semantic handles
           (cpl:mapcar-clean
            (lambda (perceived-object)
              (unless (eql (desig:desig-prop-value perceived-object 'type)
                           'desig-props::semantic-handle)
                (unless (crs:prolog `(cram-task-knowledge:perceived-object-invalid
                                      ,perceived-object))
                  perceived-object)))
            perceived-objects))
         (perceived-semantic-handles ;; Only includes semantic handles
           (cpl:mapcar-clean
            (lambda (perceived-object)
              (when (eql (desig:desig-prop-value perceived-object 'type)
                         'desig-props::semantic-handle)
                perceived-object))
            perceived-objects))
         (perceived-object-names
           (mapcar (lambda (perceived-object-designator)
                     (desig:desig-prop-value perceived-object-designator
                                             'desig-props::name))
                   perceived-object-designators))
         ;; Represents *all* objects present in the current bullet
         ;; world, except for the ones explicitly being ignored.
         (all-bullet-objects
          (cut:force-ll
           (cut:lazy-mapcar
            (lambda (bdgs)
              (cut:with-vars-bound (?o) bdgs
                ?o))
            (crs:prolog
             `(and (btr:bullet-world ?w)
                   (btr:object ?w ?o)
                   (not (btr:robot ?o))
                   (not (member ?o ,*ignored-bullet-objects*)))))))
        ;; Identify all objects that are not present at all in the
        ;; bullet world, but are identified as being seen by the
        ;; perception system.
        (unknown-and-perceived
          (cpl:mapcar-clean
           (lambda (perceived-object-name)
             (when (not (find perceived-object-name all-bullet-objects))
               perceived-object-name))
           perceived-object-names)))
    (labels ((object-name->object (object-name)
               (find object-name perceived-object-designators
                     :test (lambda (name object)
                             (string=
                              (or (when (symbolp name) (write-to-string name))
                                  name)
                              (or (when (symbolp (desig:desig-prop-value
                                                  object 'desig-props::name))
                                    (write-to-string (desig:desig-prop-value
                                                      object 'desig-props::name)))
                                  (desig:desig-prop-value
                                   object 'desig-props::name))))))
             (object-names->objects (object-names)
               (mapcar #'object-name->object object-names)))
      ;; Add objects that were not present in the currently visible
      ;; bullet world portion, but were reported by the perception
      ;; system as being visible.
      (add-appeared-objects (object-names->objects
                             unknown-and-perceived))
      (let* (;; Identify all objects that *should* be visible from the
             ;; robot's current view point.
             (should-be-visible
               (cut:force-ll
                (cut:lazy-mapcar 
                 (lambda (bdgs)
                   (cut:with-vars-bound (?o) bdgs
                     ?o))
                 (crs:prolog `(and (btr:bullet-world ?w)
                                   (btr:robot ?r)
                                   (member ?o ,all-bullet-objects)
                                   (btr:visible ?w ?r ?o))))))
             ;; Identify all objects that should be visible from the
             ;; bullet world, and are reported as being seen by the
             ;; perception system.
             (should-be-visible-and-perceived
               (cpl:mapcar-clean
                (lambda (perceived-object-name)
                  (find perceived-object-name should-be-visible))
                perceived-object-names)))
        ;; Update poses of already existing, currently visible objects
        ;; in the bullet world, based on information reported by the
        ;; perception system.
        (update-objects (object-names->objects should-be-visible-and-perceived))
        ;;(btr:simulate btr:*current-bullet-world* 10)
        (let (;; Identify all objects that should be visible from the
              ;; bullet world, but are not reported as being seen by the
              ;; perception system.
              (should-be-visible-and-not-perceived
                (cond ((desig:desig-prop-value object-template 'desig-props:name)
                       ;; If the object template has a name set, a
                       ;; specific object is being looked for. Don't
                       ;; remove other objects if that is the case, as
                       ;; they can only be wrong object detections.
                       nil)
                      (t (cpl:mapcar-clean
                          (lambda (should-be-visible-name)
                            (when (not (find should-be-visible-name
                                             perceived-object-names))
                              should-be-visible-name))
                          should-be-visible)))))
          ;; Remove objects from the current bullet world that *should* be
          ;; visible based on visibility reasoning, but aren't according
          ;; to the perception system.
          (remove-disappeared-objects should-be-visible-and-not-perceived)
          ;; Filter perceived objects based on the description of the
          ;; request (template) designator.
          (append
           (cram-task-knowledge:filter-objects
            object-template perceived-semantic-handles)
           (cram-task-knowledge:filter-objects
            object-template
            ;; Examine visible objects (new or updated) closer.
            (mapcar (lambda (examined-object-designator)
                      (let ((data (make-instance
                                   'perceived-object-data
                                   :identifier (desig:desig-prop-value
                                                examined-object-designator
                                                'desig-props::name)
                                   :object-identifier (desig:desig-prop-value
                                                       examined-object-designator
                                                       'desig-props::name)
                                   :pose (desig:desig-prop-value
                                          (desig:desig-prop-value
                                           examined-object-designator 'desig-props::at)
                                          'desig-props::pose))))
                        (desig:make-effective-designator
                         object-template
                         :new-properties (desig:description examined-object-designator)
                         :data-object data)))
                    (mapcar (lambda (perceived-object-designator)
                              (cram-task-knowledge:examine-perceived-object-designator
                               object-template perceived-object-designator))
                            perceived-object-designators)))))))))
