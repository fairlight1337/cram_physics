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

(def-fact-group bullet-world-facts ()
  
  (<- (bullet-world ?world)
    (instance-of bt-reasoning-world ?world))

  (<- (bullet-world ?world ?obj)
    ;; The world ?obj belongs to
    (get-slot-value ?obj world ?world))

  (<- (assert-object ?world ?object-type ?name ?pose . ?args)
    (lisp-fun apply add-object
              ?world ?object-type
              ?name ?pose ?args
              ?_))

  (<- (retract-object ?world ?name)
    (lisp-fun remove-object ?world ?name ?_))

  (<- (object ?world ?name ?obj)
    (bound ?name)
    (not (bound ?obj))
    (lisp-fun object ?world ?name ?obj))

  (<- (object ?world ?name ?obj)
    (bound ?obj)
    (bullet-world ?world ?obj)
    (lisp-fun name ?obj ?name))

  (<- (object ?world ?name ?obj)
    (not (bound ?name))
    (not (bound ?obj))
    (lisp-fun find-objects ?world ?objs)
    (member ?obj ?objs)
    (lisp-fun name ?obj ?name))
  
  ;; Performs one simulation step in the world
  (<- (step ?world)
    (step ?world 0.01))

  ;; Performs one simulation step of length ?dt in the world
  (<- (step ?world ?dt)
    (lisp-fun step-simulation ?world ?dt))

  ;; Simulates the next ?t seconds
  (<- (simulate ?world ?t)
    (lisp-fun simulate ?world ?t ?_))

  ;; Simulates the next ?t seconds with a step of ?dt
  (<- (simulate ?world ?t ?dt)
    (lisp-fun simulate ?world ?t ?dt ?_))

  (<- (simulate-realtime ?world ?t)
    (lisp-fun simulate ?world ?t 0.01 :realtime ?_)))

(def-fact-group poses ()

  (<- (pose ?obj ?pose)
    (bound ?obj)
    (not (bound ?pose))
    (lisp-fun pose ?obj ?pose))

  (<- (pose ?obj ?pose)
    (bound ?obj)
    (bound ?pose)
    (pose ?obj ?obj-pose)
    (poses-equal ?pose ?obj-pose 0.01 0.01))

  (<- (assert-object-pose ?obj ?pose)
    (bound ?obj)
    (bound ?pose)
    (lisp-fun set-object-pose ?obj ?pose ?_))

  (<- (pose ?obj ?position ?orientation)
    (bound ?obj)
    (instance-of object ?obj)
    (pose ?obj ?p)
    (position ?p ?position)
    (orientation ?p ?orientation))

  (<- (pose ?pose ?position ?orientation)
    (bound ?pose)
    (position ?pose ?position)
    (orientation ?pose ?orientation))

  (<- (pose ?pose (?x ?y ?z) (?ax ?ay ?az ?aw))
    (not (bound ?pose))
    (ground (?x ?y ?z))
    (ground (?ax ?ay ?az ?aw))
    (lisp-fun cl-transforms:make-3d-vector ?x ?y ?z ?o)
    (lisp-fun cl-transforms:make-quaternion ?ax ?ay ?az ?aw ?q)
    (lisp-fun cl-transforms:make-pose ?o ?q ?pose))
  
  (<- (pose ?pose ?position (?ax ?ay ?az ?aw))
    (not (bound ?pose))
    (lisp-type ?position cl-transforms:3d-vector)
    (ground (?ax ?ay ?az ?aw))
    (lisp-fun cl-transforms:make-quaternion ?ax ?ay ?az ?aw ?q)
    (lisp-fun cl-transforms:make-pose ?position ?q ?pose))

  (<- (pose ?pose (?x ?y ?z) ?orientation)
    (not (bound ?pose))
    (ground (?x ?y ?z))
    (lisp-type ?orientation cl-transforms:quaternion)
    (lisp-fun cl-transforms:make-3d-vector ?x ?y ?z ?o)
    (lisp-fun cl-transforms:make-pose ?position ?orientation ?pose))

  (<- (position ?pose (?x ?y ?z))
    (lisp-fun cl-transforms:origin ?pose ?p)
    (lisp-fun cl-transforms:x ?p ?x)
    (lisp-fun cl-transforms:y ?p ?y)
    (lisp-fun cl-transforms:z ?p ?z))

  (<- (orientation ?pose (?x ?y ?z ?w))
    (lisp-fun cl-transforms:orientation ?pose ?o)
    (lisp-fun cl-transforms:x ?o ?x)
    (lisp-fun cl-transforms:y ?o ?y)
    (lisp-fun cl-transforms:z ?o ?z)
    (lisp-fun cl-transforms:w ?o ?w))

  (<- (poses-equal ?pose-1 ?pose-2 (?dist-sigma ?ang-sigma))
    (lisp-pred poses-equal-p ?pose-1 ?pose-2 ?dist-sigma ?ang-sigma))

  (<- (random-poses-on ?bottom ?top ?poses)
    (ground (?bottom ?top))
    (not (bound ?poses))
    (generate ?poses (random-poses-on ?bottom ?top)))

  (<- (random-poses-on ?n ?bottom ?top ?poses)
    (ground (?bottom ?top))
    (not (bound ?poses))
    (generate ?tmp (random-poses-on ?bottom ?top))
    (take ?n ?tmp ?poses))

  (<- (n-poses-on ?n ?bottom ?top ?poses)
    (ground (?bottom ?top ?n))
    (not (bound ?poses))
    (generate ?poses (n-poses-on ?bottom ?top ?n))))

(def-fact-group robot-model ()

  (<- (link-pose ?robot ?name ?pose)
    (bound ?robot)
    (bound ?name)
    (-> (bound ?pose)
        (and
         (lisp-fun link-pose ?robot ?name ?l-p)
         (poses-equal ?pose ?l-p 0.01 0.01))
        (lisp-fun link-pose ?robot ?name ?pose)))

  (<- (link-pose ?robot ?name ?pose)
    (bound ?robot)
    (not (bound ?name))
    (lisp-fun link-names ?robot ?names)
    (member ?name ?names)
    (link-pose ?robot ?name ?pose)))

(def-fact-group force-dynamic-states ()

  (<- (contact ?world ?obj-1 ?obj-2)
    (bound ?obj-1)
    (bound ?obj-2)
    (lisp-fun perform-collision-detection ?world ?_)
    (lisp-pred contact-p ?world ?obj-1 ?obj-2))

  (<- (contact ?world ?obj-1 ?obj-2)
    (bound ?obj-1)
    (not (bound ?obj-2))
    (lisp-fun perform-collision-detection ?world ?_)
    (lisp-fun find-objects-in-contact ?world ?obj-1 ?objs)
    (member ?obj-2 ?objs))

  (<- (contact ?world ?obj-1 ?obj-2)
    (not (bound ?obj-1))
    (bound ?obj-2)
    (contact ?world ?obj-2 ?obj-1))

  (<- (contact ?world ?obj-1 ?obj-2)
    (not (bound ?obj-1))
    (not (bound ?obj-2))
    (lisp-fun perform-collision-detection ?world ?_)
    (lisp-fun find-all-contacts ?world ?contacts)
    (member (?obj-1 ?obj-2) ?contacts))

  (<- (stable ?world ?obj)
    (object ?world ?_ ?obj)
    (pose ?obj ?pose-1)
    (with-stored-world ?world
      (simulate ?world 0.5)
      ;; checking for active-tag does not always work and requires
      ;; pretty long simlation times. Additionally some bodies are
      ;; just very unstable and never get deactivated, e.g. cylinders.
      ;;
      ;; (lisp-pred stable-p ?obj)
      (pose ?obj ?pose-2))
    (poses-equal ?pose-1 ?pose-2 (0.01 0.03)))

  (<- (stable ?world)
    (forall (object ?world ?_ ?o)
            (stable ?world ?o)))

  (<- (supported-by ?world ?top ?bottom)
    (object ?world ?tn ?top)
    (object ?world ?bn ?bottom)
    (above ?top ?bottom)
    (contact ?world ?top ?bottom)
    (stable ?world ?top)))

(def-fact-group spatial-relations ()

  (<- (above ?obj-1 ?obj-2)
    (bound ?obj-1)
    (bound ?obj-2)
    (lisp-pred above-p ?obj-1 ?obj-2))

  (<- (above ?obj-1 ?obj-2)
    (not (bound ?obj-1))
    (bound ?obj-2)
    (lisp-fun find-objects-above ?obj-2 ?objs)
    (member ?obj-1 ?objs))

  (<- (above ?obj-1 ?obj-2)
    (bound ?obj-1)
    (not (bound ?obj-2))
    (lisp-fun find-objects-below ?obj-1 ?objs)
    (member ?obj-2 ?objs))

  (<- (below ?obj-1 ?obj-2)
    (bound ?obj-1 ?obj-2)
    (lisp-pred below-p ?obj-1 ?obj-2))

  (<- (below ?obj-1 ?obj-2)
    (not (bound ?obj-1))
    (bound ?obj-2)
    (lisp-fun find-objects-below ?obj-2 ?objs)
    (member ?obj-1 ?objs))

  (<- (below ?obj-1 ?obj-2)
    (bound ?obj-1)
    (not (bound ?obj-2))
    (lisp-fun find-objects-above ?obj-1 ?objs)
    (member ?obj-2 ?objs)))

(def-fact-group visibility ()
  (<- (visible ?world ?camera-pose ?obj)
    (ground (?world ?camera-pose))
    (object ?world ?_ ?obj)
    (lisp-pred object-visible-p ?world ?camera-pose ?obj))

  (<- (occluding-objects ?world ?camera-pose ?obj ?objs)
    (ground (?world ?camera-pose))
    (object ?world ?_ ?obj)
    (lisp-fun occluding-objects ?world ?camera-pose ?obj ?objs))

  (<- (occluding-object ?world ?camera-pose ?obj ?occluding-obj)
    (occluding-objects ?world ?camera-pose ?obj ?objs)
    (member ?occluding-obj ?objs)))

(def-fact-group reachability ()
  (<- (grasp :top))
  (<- (grasp :side))  

  (<- (side :right))
  (<- (side :left))

  (<- (reachable ?w ?robot ?obj)
    (once (reachable ?w ?robot ?obj ?_)))

  (<- (reachable ?w ?robot ?obj ?side)
    (ground (?w ?robot ?obj))
    (side ?side)
    (once
     (grasp ?g)
     (lisp-pred object-reachable-p ?robot ?obj :side ?side :grasp ?g)))

  (<- (blocking ?w ?robot ?obj ?objs)
    (blocking ?w ?robot ?obj ?side ?objs))

  (<- (blocking ?w ?robot ?obj ?side ?objs)
    (ground (?w ?robot))
    (object ?w ?_ ?obj)
    (side ?side)
    (-> (setof
         ?o
         (and
          (grasp ?grasp)
          ;; We don't want to have the supporting object as a blocking
          ;; object.
          (supported-by ?w ?obj ?supporting)
          ;; Generate all ik solutions
          (lisp-fun reach-object-ik ?robot ?obj :side ?side :grasp ?grasp ?ik-solutions)
          (member ?ik-solution ?ik-solutions)
          (ik-solution-in-collision ?w ?robot ?ik-solution ?colliding-objects)
          (member ?o ?colliding-objects)
          (not (== ?o ?obj))
          (not (== ?o ?supporting)))
         ?objs)
        (true)
        (== ?objs ())))
  
  (<- (ik-solution-in-collision ?w ?robot ?ik-solution ?colliding-objects)
    (with-stored-world ?w
      (lisp-fun set-robot-state-from-joints ?ik-solution ?robot ?_)
      (findall ?obj (contact ?w ?robot ?obj) ?colliding-objects))))

(def-fact-group debug ()
  (<- (debug-window ?world)
    (lisp-fun add-debug-window ?world ?_)))
