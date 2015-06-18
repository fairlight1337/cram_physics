;;; Copyright (c) 2015, Jan Winkler <winkler@cs.uni-bremen.de>
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
;;;     * Neither the name of University of Bremen nor the names of its
;;;       contributors may be used to endorse or promote products derived from
;;;       this software without specific prior written permission.
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

(in-package :btr)

(defvar *ignored-perception-bullet-objects* nil)

(defun ignore-perception-bullet-object (object-name)
  (setf *ignored-perception-bullet-objects*
        (remove object-name *ignored-perception-bullet-objects*))
  (push object-name *ignored-perception-bullet-objects*))

(defmethod cram-task-knowledge:objects-perceived (object-designators)
  ;; Make sure that the current pose and everything is in the
  ;; bullet reasoning beliefstate.
  (plan-knowledge:on-event
   (make-instance 'plan-knowledge:robot-state-changed))
  (let* ((perceived-objects object-designators)
         (perceived-object-designators ;; Doesn't include semantic handles
           (cpl:mapcar-clean
            (lambda (perceived-object)
              (unless (eql (desig-prop-value perceived-object 'type)
                           'desig-props::semantic-handle)
                (unless (crs:prolog `(perceived-object-invalid
                                      ,perceived-object))
                  perceived-object)))
            perceived-objects))
         (perceived-semantic-handles ;; Only includes semantic handles
           (cpl:mapcar-clean
            (lambda (perceived-object)
              (when (eql (desig-prop-value perceived-object 'type)
                         'desig-props::semantic-handle)
                perceived-object))
            perceived-objects))
         (perceived-object-names
           (mapcar (lambda (perceived-object-designator)
                     (desig-prop-value perceived-object-designator 'name))
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
                   (not (member ?o ,*ignored-perception-bullet-objects*)))))))
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
                             (eql name (desig:desig-prop-value
                                        object 'name)))))
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
                (cpl:mapcar-clean
                 (lambda (should-be-visible-name)
                   (when (not (find should-be-visible-name
                                    perceived-object-names))
                     should-be-visible-name))
                 should-be-visible)))
          ;; Remove objects from the current bullet world that *should* be
          ;; visible based on visibility reasoning, but aren't according
          ;; to the perception system.
          (remove-disappeared-objects should-be-visible-and-not-perceived)
          ;; Filter perceived objects based on the description of the
          ;; request (template) designator.
          (append
           (filter-perceived-objects
            object-designator perceived-semantic-handles)
           (filter-perceived-objects
            object-designator
            ;; Examine visible objects (new or updated) closer.
            (mapcar (lambda (examined-object-designator)
                      (let ((data (make-instance
                                   'perceived-object-data
                                   :identifier (desig-prop-value
                                                examined-object-designator 'name)
                                   :object-identifier (desig-prop-value
                                                       examined-object-designator 'name)
                                   :pose (desig-prop-value
                                          (desig-prop-value
                                           examined-object-designator 'desig-props::at)
                                          'desig-props::pose))))
                        (make-effective-designator
                         object-designator
                         :new-properties (description examined-object-designator)
                         :data-object data)))
                    (mapcar (lambda (perceived-object-designator)
                              (examine-perceived-object-designator
                               object-designator perceived-object-designator))
                            perceived-object-designators)))))))))

(defmethod examine-perceived-object-designator
    ((original-designator desig:object-designator)
     (object-designator desig:object-designator))
  "Enriches a perceived object designator `object-designator' with
additional information from the reasoning system. First, the type is
infered (if not set already either manually in the requesting
designator, or the one returned from the perception system), and then
additional properties are infered and appended to the designator's
description."
  (let* ((object-description (desig:description
                              object-designator))
         (type (or (desig:desig-prop-value object-designator
                                           'desig-props:type)
                   (cut:with-vars-bound (?type)
                       (first
                        (crs:prolog `(infer-object-property
                                      ,object-designator
                                      desig-props:type ?type)))
                     (unless (eql ?type '?type)
                       ?type))
                   (desig:desig-prop-value original-designator
                                           'desig-props:type)))
         (typed-object-designator
           (or (and (or (eql type '?type) (not type)) object-designator)
               (desig:make-designator
                'object
                (append
                 (remove-if (lambda (x) (eql (car x) 'type))
                            object-description)
                 `((type ,type)))
                object-designator)))
         (new-properties
           (cut:force-ll (cut:lazy-mapcar
                          (lambda (bdgs)
                            (cut:with-vars-bound (?key ?value) bdgs
                              `(,?key ,?value)))
                          (crs:prolog `(infer-object-property
                                        ,typed-object-designator
                                        ?key ?value)))))
         (refined-old
           (remove-if (lambda (x)
                        (find x new-properties
                              :test (lambda (x y)
                                      (eql (car x) (car y)))))
                      (desig:description
                       typed-object-designator))))
    (let* ((infered-description (append refined-old new-properties))
           (complete-description
             (let ((original-description
                     (desig:description original-designator)))
               (append infered-description
                       (cpl:mapcar-clean
                        (lambda (original-property)
                          (unless (find original-property
                                        infered-description
                                        :test (lambda (x y)
                                                (eql (car x) (car y))))))
                        original-description)))))
      (desig:make-designator
       'object complete-description object-designator))))

(defun remove-disappeared-objects (object-names)
  "Removes objects from the current bullet world. They are refered to by the `name' property specified in their designator."
  (dolist (object-name object-names)
    (crs:prolog
     `(and (btr:bullet-world ?w)
           (btr:retract
            (btr:object
             ?w ,object-name))))
    (make-instance 'plan-knowledge:object-removed-event
                   :object-name object-name)))

(defun add-appeared-objects (objects)
  "Adds objects to the current bullet world. In the world, they then consist of boxes of dimensions as specified in the `dimensions'
property in their designator."
  (dolist (object objects)
    (let ((pose (desig:desig-prop-value
                 (desig:desig-prop-value object 'at)
                 'pose))
          (dimensions (desig:desig-prop-value object 'dimensions))
          (name (desig:desig-prop-value object 'name)))
      ;; TODO(winkler): Fix me
      ;; (crs:prolog `(and (btr:bullet-world ?w)
      ;;                   (btr:assert
      ;;                    (btr:object
      ;;                     ?w btr:box ,name ,pose
      ;;                     :mass 0.1
      ;;                     :size ,(map 'list #'identity dimensions)))))
      (crs:prolog
       `(and (bullet-world ?w)
             (assert
              (object
               ?w btr::mesh ,name ,pose
               :mass 0.1
               :mesh desig-props::mondamin :color (0.8 0.4 0.2))))))
    (make-instance 'plan-knowledge:object-perceived-event
                   :object-designator object
                   :perception-source 'generic)))

(defun update-objects (objects)
  "Updates objects' poses in the current bullet world based on the
`name', and the `pose' properties in their respective designator."
  (dolist (object objects)
    (let ((pose (desig:desig-prop-value
                 (desig:desig-prop-value object 'at)
                 'pose))
          (name (desig:desig-prop-value object 'name)))
      (crs:prolog
       `(and (bullet-world ?w)
             (assert
              (object-pose
               ?w ,name ,pose)))))
    (make-instance 'plan-knowledge:object-updated-event
                   :object-designator object)))
