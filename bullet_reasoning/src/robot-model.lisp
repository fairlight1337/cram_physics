;;;
;;; Copyright (c) 2010, Lorenz Moesenlechner <moesenle@in.tum.de>
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
;;;

(in-package :btr)

(defvar *robot-model-alpha* nil)

(defgeneric urdf-make-collision-shape (geometry &optional color))

(defmethod urdf-make-collision-shape ((box cl-urdf:box) &optional (color '(0.8 0.8 0.8 1.0)))
  (make-instance 'colored-box-shape
    :half-extents (cl-transforms:v*
                   (cl-urdf:size box) 0.5)
    :color (apply-aplha-value color)))

(defmethod urdf-make-collision-shape ((cylinder cl-urdf:cylinder) &optional (color '(0.8 0.8 0.8 1.0)))
  (make-instance 'colored-cylinder-shape
    :half-extents (cl-transforms:make-3d-vector
                   (cl-urdf:radius cylinder)
                   (cl-urdf:radius cylinder)
                   (* 0.5 (cl-urdf:cylinder-length cylinder)))
    :color (apply-aplha-value color)))

(defmethod urdf-make-collision-shape ((sphere cl-urdf:sphere) &optional (color '(0.8 0.8 0.8 1.0)))
  (make-instance 'colored-sphere-shape :radius (cl-urdf:radius sphere)
    :color (apply-aplha-value color)))

(defmethod urdf-make-collision-shape ((mesh cl-urdf:mesh) &optional (color '(0.8 0.8 0.8 1.0)))
  (let ((model (load-mesh mesh)))
    (make-instance 'convex-hull-mesh-shape
                   :color (apply-aplha-value color)
                   :faces (physics-utils:3d-model-faces model)
                   :points (physics-utils:3d-model-vertices model))))

(defstruct collision-information
  rigid-body-name flags)

(defstruct attachment
  "Represents a link between an object and a link. `object' must be an
instance of class OBJECT. `link' must be a string, the name of the
link. If `loose' is non-NIL, it means that if the link moves, the pose
of the object should _not_ be updated."
  (object nil)
  (link "" :type string)
  (loose nil :type (or nil t)))

(defclass robot-object (object)
  ((links :initarg :links :initform (make-hash-table :test 'equal) :reader links)
   (joint-states :initarg :joint-states :initform (make-hash-table :test 'equal)
                 :reader joint-states)
   (urdf :initarg :urdf :reader urdf)
   (attached-objects
    :initarg :attached-objects :initform nil
                     :reader attached-objects
                     :documentation "An alist that maps object
                     instances to a list of instances of the struct
                     `attachment' and an instance of
                     `collision-information'.")
   (initial-pose :initarg :pose
                 :documentation "Pose that got passed in initially. It
                 is returned by the `pose' method if `reference-body'
                 is invalid.")))

(defgeneric joint-names (robot-object)
  (:documentation "Returns the list of joints")
  (:method ((robot-object robot-object))
    (loop for name being the hash-keys in (joint-states robot-object)
          collecting name)))

(defgeneric joint-state (robot-object name)
  (:documentation "Returns the value of the joint named `name'"))

(defgeneric (setf joint-state) (new-value robot-object name)
  (:documentation "Sets the specific joint to a new value and updates
  all child-link positions"))

(defgeneric link-names (robot-object)
  (:documentation "Returns a list of link names")
  (:method ((robot-object robot-object))
    (loop for name being the hash-keys in (links robot-object)
          collecting name)))

(defgeneric link-pose (robot-object name)
  (:documentation "Gets the pose of a link"))

(defgeneric (setf link-pose) (new-value robot-object name)
  (:documentation "Sets the pose of a link and all its children"))

(defgeneric object-attached (robot-object object)
  (:documentation "Returns the list of links `object' has been
  attached to.")
  (:method ((robot-object robot-object) (object object))
    (with-slots (attached-objects) robot-object
      (mapcar #'attachment-link (car (cdr (assoc (name object) attached-objects)))))))

(defgeneric attach-object (robot-object obj link &key loose)
  (:documentation "Adds `obj' to the set of attached objects. If
  `loose' is set to NIL and the link the object is attached to is
  moved, the object moves accordingly.")
  (:method ((robot-object robot-object) (obj object) link &key loose)
    (unless (gethash link (links robot-object))
      (error 'simple-error :format-control "Link ~a unknown"
             :format-arguments (list link)))
    (with-slots (attached-objects) robot-object
      (let ((obj-attachment (assoc (name obj) attached-objects))
            (new-attachment
              (make-attachment
               :object (name obj) :link link :loose loose)))
        (cond (obj-attachment
               (pushnew new-attachment (car (cdr obj-attachment))
                        :test #'equal :key #'attachment-link))
              (t
               (push (cons (name obj)
                           (cons
                            (list new-attachment)
                            (loop for body in (rigid-bodies obj)
                                  collecting (make-collision-information
                                              :rigid-body-name (cl-bullet:name body)
                                              :flags (collision-flags body))
                                  do (setf (collision-flags body) :cf-static-object))))
                     attached-objects)))))))

(defgeneric detach-object (robot-object obj &optional link)
  (:documentation "Detaches `obj' from the set of attached objects. If
  `link' is specified, detaches the object only from
  `link'. Otherwise, detaches `obj' from all links.")
  (:method ((robot-object robot-object) (obj object) &optional link)
    (flet ((reset-collision-information (object collision-information)
             (loop for collision-data in collision-information
                   for body = (rigid-body
                               object (collision-information-rigid-body-name
                                       collision-data))
                   do (setf (collision-flags body)
                            (collision-information-flags collision-data)))))
      (with-slots (attached-objects) robot-object
        (let ((attachment (assoc (name obj) attached-objects)))
          (cond (link
                 (setf (car (cdr attachment))
                       (remove link (car (cdr attachment))
                               :test #'equal :key #'attachment-link))
                 (unless (cdr attachment)
                   (setf attached-objects (remove (name obj) attached-objects
                                                  :key #'car))
                   (reset-collision-information obj (cdr (cdr attachment)))))
                (t (setf attached-objects (remove (name obj) attached-objects
                                                  :key #'car))
                   (reset-collision-information obj (cdr (cdr attachment))))))))))

(defgeneric detach-all-objects (robot-object)
  (:documentation "Removes all objects form the list of attached
  objects.")
  (:method ((robot-object robot-object))
    (dolist (attached-object (attached-objects robot-object))
      (detach-object robot-object
                     (object *current-bullet-world* (car attached-object))))))

(defgeneric gc-attached-objects (robot-object)
  (:documentation "Removes all attached objects with an invalid world
  instance")
  (:method ((robot-object robot-object))
    (with-slots (attached-objects) robot-object
      (setf attached-objects
            (remove-if-not (lambda (obj)
                             (slot-value (car obj) 'world))
                           attached-objects)))))

(defgeneric attach-contacting-objects (robot-object
                                       &key blacklist test
                                         detach-invalid)
  (:documentation "Attaches all objects of `world' that are in contact
  to `robot-object' and not a member of the list `blacklist' or `test'
  fails. `test' must be a function with exactly two parameters, the
  object to be attached and the link name it is contacting, and
  returns a generalized boolean indicating if the object should be
  attached. If `detach-invalid' is non-NIL, it is asured that a
  contacting object is only attached to its contacting links.")
  (:method ((robot-object robot-object)
            &key blacklist (test (constantly t))
              detach-invalid)
    (let ((detached-cache (make-hash-table)))
      (gc-attached-objects robot-object)
      (loop for (obj . link-name) in (link-contacts robot-object)
            when (and (not (member obj blacklist)) (funcall test obj link-name)) do
              (when (and detach-invalid (not (gethash obj detached-cache)))
                (setf (gethash obj detached-cache) t)
                (detach-object robot-object obj))
              (attach-object robot-object obj link-name)))))

(defmethod initialize-instance :after ((robot-object robot-object)
                                       &key color name pose
                                         (collision-group :character-filter)
                                         (collision-mask '(:default-filter :static-filter)))
  (with-slots (rigid-bodies links urdf pose-reference-body) robot-object
    (labels ((make-link-bodies (pose link)
               "Returns the list of rigid bodies of `link' and all its sub-links"
               (let* ((pose-transform (cl-transforms:reference-transform pose))
                      (collision-elem (cl-urdf:collision link))
                      (bodies (mapcan (lambda (joint)
                                        (make-link-bodies (cl-transforms:transform-pose
                                                           pose-transform (cl-urdf:origin joint))
                                                          (cl-urdf:child joint)))
                                      (cl-urdf:to-joints link))))
                 (if collision-elem
                     (cons (cons
                            (cl-urdf:name link)
                            (make-instance
                                'rigid-body
                              :name (make-rigid-body-name name (cl-urdf:name link))
                              :mass 0
                              :pose (cl-transforms:transform-pose
                                     pose-transform (cl-urdf:origin collision-elem))
                              :collision-shape (urdf-make-collision-shape
                                                (cl-urdf:geometry collision-elem)
                                                (apply-aplha-value
                                                 (or (when (and (cl-urdf:visual link)
                                                                (cl-urdf:material (cl-urdf:visual link)))
                                                       (cl-urdf:color (cl-urdf:material (cl-urdf:visual link))))
                                                     color)))
                              :collision-flags :cf-default
                              :group collision-group
                              :mask collision-mask))
                           bodies)
                     bodies))))
      (let ((bodies (make-link-bodies pose (cl-urdf:root-link urdf))))
        (initialize-rigid-bodies robot-object (mapcar #'cdr bodies))
        (setf pose-reference-body (make-rigid-body-name name (cl-urdf:name (cl-urdf:root-link urdf))))
        (loop for (name . body) in bodies do
          (setf (gethash name links) body))
        (loop for name being the hash-keys in (cl-urdf:joints urdf) do
          (setf (gethash name (joint-states robot-object)) 0.0d0))))))

(defgeneric link-contacts (robot-object)
  (:method ((robot-object robot-object))
    (flet ((find-link-name (body)
             (loop for name being the hash-keys in (links robot-object)
                   using (hash-value rb) do
                     (when (eq body rb)
                       (return name)))))
      (with-slots (world) robot-object
        (let ((objects (objects world))
              (contacts nil))
          (perform-collision-detection world)
          (dolist (manifold (contact-manifolds world) contacts)
            (let ((obj (loop for obj in objects
                             when (and (rigid-body obj (bt:name (body-1 manifold)))
                                       (rigid-body robot-object (bt:name (body-2 manifold))))
                               do (return (cons obj (body-2 manifold)))
                             when (and (rigid-body obj (bt:name (body-2 manifold)))
                                       (rigid-body robot-object (bt:name (body-1 manifold))))
                               do (return (cons obj (body-1 manifold)))
                             finally (return nil))))
              (when obj
                (push (cons (car obj) (find-link-name (cdr obj)))
                      contacts)))))))))

(defmethod copy-object ((obj robot-object) (world bt-reasoning-world))
  (with-slots (links joint-states urdf) obj
    (change-class
     (call-next-method) 'robot-object
     :links (copy-hash-table links)
     :joint-states (copy-hash-table joint-states)
     :urdf urdf
     :pose (slot-value obj 'initial-pose)
     :attached-objects (copy-list (attached-objects obj)))))

(defmethod add-object ((world bt-world) (type (eql 'urdf)) name pose
                       &key urdf (color '(0.8 0.8 0.8 1.0)))
  (make-instance 'robot-object
    :name name
    :world world
    :pose (ensure-pose pose)
    :urdf (etypecase urdf
            (cl-urdf:robot urdf)
            (string (handler-bind ((cl-urdf:urdf-type-not-supported #'muffle-warning))
                      (cl-urdf:parse-urdf urdf))))
    :color color))

(defun update-attached-object-poses (robot-object link pose)
  "Updates the poses of all objects that are attached to
`link'. `pose' is the new pose of `link'"
  (with-slots (attached-objects links) robot-object
    (let ((body (gethash (cl-urdf:name link) links)))
      (when body
        (let* ((attachments
                 (mapcar
                  #'car
                  (remove-if-not
                   (lambda (attachment)
                     (let ((link-attachment
                             (find (cl-urdf:name link) (car (cdr attachment))
                                   :key #'attachment-link :test #'equal)))
                       (and link-attachment
                            (not (attachment-loose link-attachment)))))
                   attached-objects)))
               (body-transform (cl-transforms:reference-transform (cl-bullet:pose body)))
               (pose-transform (cl-transforms:reference-transform pose))
               (pose-delta (cl-transforms:transform*
                            (cl-transforms:transform*
                             pose-transform
                             (cl-urdf:origin (cl-urdf:collision link)))
                            (cl-transforms:transform-inv body-transform))))
          (dolist (attachment attachments)
            (let ((attached-object (object (world robot-object) attachment)))
              (when attached-object
                (setf (pose attached-object)
                      (cl-transforms:transform-pose pose-delta (pose attached-object)))))))))))

(defun update-link-poses (robot-object link pose)
  "Updates the pose of `link' and all its children according to
current joint states"
  (let ((links (links robot-object))
        (joint-states (joint-states robot-object)))
    (let ((body (gethash (cl-urdf:name link) links))
          (pose-transform (cl-transforms:reference-transform pose)))
      (when body
        (update-attached-object-poses robot-object link pose)
        (setf (cl-bullet:pose body) (cl-transforms:transform-pose
                           pose-transform
                           (cl-urdf:origin (cl-urdf:collision link)))))
      (dolist (to-joint (cl-urdf:to-joints link))
        (update-link-poses robot-object
                           (cl-urdf:child to-joint)
                           (cl-transforms:transform-pose
                            pose-transform
                            (cl-transforms:transform-pose
                             (cl-urdf:origin to-joint)
                             (joint-transform to-joint (gethash
                                                        (cl-urdf:name to-joint)
                                                        joint-states)))))))))

(defun joint-transform (joint value)
  (case (cl-urdf:joint-type joint)
    ((:revolute :continuous)
       (cl-transforms:make-transform
        (cl-transforms:make-3d-vector 0 0 0)
        (cl-transforms:axis-angle->quaternion
         (cl-urdf:axis joint)
         value)))
    (:prismatic
       (cl-transforms:make-transform
        (cl-transforms:v* (cl-urdf:axis joint) value)
        (cl-transforms:make-quaternion 0 0 0 1)))
    (t (cl-transforms:make-transform
        (cl-transforms:make-3d-vector 0 0 0)
        (cl-transforms:make-quaternion 0 0 0 1)))))

(defun calculate-joint-state (obj name)
  (with-slots (urdf) obj
    (let ((links (links obj))
          (joint (gethash name (cl-urdf:joints urdf))))
      (when joint
        (let* ((parent (cl-urdf:parent joint))
               (parent-pose (find-parent-pose obj name))
               (child (cl-urdf:child joint))
               (child-body (gethash (cl-urdf:name child) links)))
          (when child-body
            (let ((origin (cl-transforms:transform*
                           (cl-transforms:reference-transform parent-pose)
                           (if (cl-urdf:collision parent)
                               (cl-transforms:transform-inv
                                (cl-transforms:reference-transform
                                 (cl-urdf:origin (cl-urdf:collision parent))))
                               (cl-transforms:make-identity-transform))
                           (cl-urdf:origin joint)))
                  (child-transform (cl-transforms:transform*
                                    (cl-transforms:reference-transform
                                     (cl-bullet:pose child-body))
                                    (cl-transforms:transform-inv
                                     (cl-transforms:reference-transform
                                      (cl-urdf:origin (cl-urdf:collision child)))))))
              (case (cl-urdf:joint-type joint)
                ((:revolute :continuous)
                 (multiple-value-bind (angle axis)
                     (cl-transforms:angle-between-quaternions
                      (cl-transforms:rotation origin)
                      (cl-transforms:rotation child-transform))
                   (if (< (cl-transforms:dot-product
                           axis (cl-urdf:axis joint))
                          0)
                       (* angle -1)
                       angle)))
                (:prismatic
                 (* (cl-transforms:v-dist
                     (cl-transforms:translation origin)
                     (cl-transforms:translation child-transform))
                    (if (< (cl-transforms:dot-product
                            (cl-transforms:v-
                             (cl-transforms:translation child-transform)
                             (cl-transforms:translation origin))
                            (cl-urdf:axis joint))
                           0)
                        -1 1)))
                (t 0.0d0)))))))))

(defmethod invalidate-object :after ((obj robot-object))
  (with-slots (world links joint-states) obj
    (loop for name being the hash-keys in links
          using (hash-value body) do
            (setf (gethash name links)
                  (rigid-body obj (name body))))
    (loop for name being the hash-keys in joint-states do
      (setf (gethash name joint-states) (or (calculate-joint-state obj name) 0.0d0)))))

(defmethod joint-state ((obj robot-object) name)
  (nth-value 0 (gethash name (joint-states obj))))

(defmethod (setf joint-state) (new-value (obj robot-object) name)
  (with-slots (urdf) obj
    (assert (gethash name (cl-urdf:joints urdf)) ()
            "Joint ~a unknown" name)
    (let* ((joint-states (joint-states obj))
           (joint (gethash name (cl-urdf:joints urdf)))
           (parent (cl-urdf:parent joint))
           (parent-pose (find-parent-pose obj name)))
      (when joint
        (let ((joint-transform
                (cl-transforms:transform*
                 (cl-transforms:reference-transform
                  (cl-urdf:origin joint))
                 (joint-transform joint new-value))))
          (setf (gethash name joint-states) new-value)
          (update-link-poses
           obj (cl-urdf:child joint)
           (cl-transforms:transform*
            (cl-transforms:reference-transform parent-pose)
            (if (cl-urdf:collision parent)
                (cl-transforms:transform-inv
                 (cl-transforms:reference-transform
                  (cl-urdf:origin (cl-urdf:collision parent))))
                (cl-transforms:make-identity-transform))
            joint-transform)))))))

(defun set-joint-state (robot name new-state)
  (setf (joint-state robot name) new-state))

(defun find-parent-pose (obj joint-name
                         &optional
                           (current-pose (cl-transforms:make-identity-pose)))
  "Tries to find the pose of the parent link of the joint named
  `joint-name'. The algorithm works as follows: if the parent link has
  a rigid body, use its pose. If not, walk the tree up, apply the
  inverse joint transform of parent's from-joint and try again."
  (with-slots (urdf links joint-states) obj
    (let* ((joint (gethash joint-name (cl-urdf:joints urdf)))
           (parent (and joint (cl-urdf:parent joint))))
      (cond ((and parent (gethash (cl-urdf:name parent) links))
             (cl-transforms:transform->pose
              (cl-transforms:transform*
               (cl-transforms:reference-transform current-pose)
               (cl-transforms:reference-transform (cl-bullet:pose (gethash (cl-urdf:name parent) links))))))
            ((and parent (cl-urdf:from-joint parent))
             ;; walk the tree up
             (let* ((parent-joint (cl-urdf:from-joint parent))
                    (parent-joint-name (cl-urdf:name parent-joint)))
               (find-parent-pose
                obj parent-joint-name
                (cl-transforms:transform->pose
                 (cl-transforms:transform*
                  (cl-transforms:reference-transform
                   (cl-urdf:origin parent-joint))
                  (joint-transform
                   parent-joint
                   (or (joint-state obj parent-joint-name)
                       0.0))
                  (cl-transforms:reference-transform current-pose))))))
            (t ;; We are at the root. Return the object's inverse pose
               ;; multiplied with current-pose
             (cl-transforms:transform*
              (cl-transforms:reference-transform (pose obj))
              (cl-transforms:reference-transform current-pose)))))))

(defmethod link-pose ((obj robot-object) name)
  ;; We need to handle two different cases here. One is when we have a
  ;; rigid body for a specific link. Then reading the pose is just
  ;; returning the pose of the body minus the pose of the collision
  ;; object in the body. The second case is a link without a rigid
  ;; body, i.e. without a collision model. We need to walk up the tree
  ;; until we reach a body, collecting the transforms along that path.
  (with-slots (links urdf) obj
    (let ((link (gethash name (cl-urdf:links urdf)))
          (body (gethash name links)))
      (cond (body
             (cl-transforms:transform->pose
              (cl-transforms:transform*
               (cl-transforms:reference-transform
                (cl-bullet:pose body))
               (cl-transforms:transform-inv
                (cl-transforms:reference-transform
                 (cl-urdf:origin (cl-urdf:collision link)))))))
            ((and link (cl-urdf:from-joint link)
                  (cl-urdf:parent (cl-urdf:from-joint link)))
             (cl-transforms:transform-pose
              (cl-transforms:reference-transform
               (link-pose obj (cl-urdf:name (cl-urdf:parent
                                             (cl-urdf:from-joint link)))))
              (cl-urdf:origin (cl-urdf:from-joint link))))
            (;; We are at the root of the tree
             link (slot-value obj 'initial-pose))
            (t nil)))))

(defmethod (setf link-pose) (new-value (obj robot-object) name)
  (with-slots (urdf) obj
    (let* ((links (links obj))
           (link (gethash name (cl-urdf:links urdf)))
           (body (gethash name links)))
      ;; Should we throw an error here?
      (when body
        (update-link-poses obj link new-value)
        (let ((joint (cl-urdf:from-joint link)))
          (when joint
            (setf (gethash (cl-urdf:name joint) (joint-states obj))
                  (calculate-joint-state obj (cl-urdf:name joint)))))))))

(defmethod pose ((obj robot-object))
  (with-slots (urdf) obj
    (link-pose obj (cl-urdf:name (cl-urdf:root-link urdf)))))

(defmethod (setf pose) (new-value (obj robot-object))
  (with-slots (urdf) obj
    (setf (link-pose obj (cl-urdf:name (cl-urdf:root-link urdf)))
          new-value)))

(defmacro with-alpha (alpha &body body)
  `(let ((*robot-model-alpha* ,alpha))
     ,@body))

(defun apply-aplha-value (color)
  (destructuring-bind (r g b a) color
    (list r g b (or *robot-model-alpha* a))))

(defun load-mesh (mesh)
  "Loads and resizes the 3d-model"
  (let ((model (physics-utils:load-3d-model (physics-utils:parse-uri (cl-urdf:filename mesh)))))
      (cond ((cl-urdf:scale mesh) (physics-utils:scale-3d-model model (cl-urdf:scale mesh)))
            ((cl-urdf:size mesh) (physics-utils:resize-3d-model model (cl-urdf:size mesh)))
            (t model))))
