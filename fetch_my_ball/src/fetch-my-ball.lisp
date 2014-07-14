(in-package :fetch-my-ball)

(defvar *wheelbase* 0.115)
(defvar *wheel-radius* 0.027)
(defvar *wheel-circumference* (* 2 pi *wheel-radius*))
(defvar *pub*)
(defvar *border-patrol-subscriber*)
(defvar *joint-state-subscriber-hash-table* (make-hash-table))
(defvar *normal-r* 0.0)
(defvar *normal-g* 0.0)
(defvar *normal-b* 0.0)
(defvar *avg-color-lock* (make-lock))
(defvar *avg-color* '(nil nil nil nil nil nil))
(defvar *suspected-crossing* nil)
(defvar *crossing* nil)
(defvar *crossing-lock* (make-lock))
(defvar *l-motor-joint-state*)
(defvar *r-motor-joint-state*)
(defvar *gripper-state*)
(defvar *avg-range-lock* (make-lock))
(defvar *avg-range* '(nil nil nil nil nil nil))
(defvar *pre-grasp-range* 0.075)
(defvar *grasp-range* 0.02)
(defvar *empty-gripper-range* 0.0799999982119)
(defvar *reached-target-area* nil)

(defun init-driver ()
  (format t "Starting up ros.~%")
  (roslisp-utilities:startup-ros)
  (format t "Subscribing to /joint_state.~%")
  (subscribe "joint_state" "sensor_msgs/JointState" (lambda (val)
    (with-fields ((name name)) val
      (cond ((string= (elt name 0) "l_motor_joint") (setf *l-motor-joint-state* val))
            ((string= (elt name 0) "r_motor_joint") (setf *r-motor-joint-state* val))
            ((string= (elt name 0) "gripper") (setf *gripper-state* val))))))
  #|
  (format t "Starting up 1d filter.~%")
  (1d-filter::filter-smooth)
  (sleep 1)
  (format t "Subscribing to /avg_range.~%")
  (subscribe "/avg_range" "std_msgs/Float64"
             (lambda (msg)
               (with-fields ((data data)) msg
                 (setf *avg-range* data))))
  |#
  (format t "Subscribing to /ultrasonic_sensor.~%")
  (subscribe "/ultrasonic_sensor" "sensor_msgs/Range"
             #'(lambda (msg)
                 (with-fields ((range range)) msg
                   (with-lock-held (*avg-range-lock*)
                     (push (if (< range 1.2)
                               range
                               nil)
                           *avg-range*)
                     (setf *avg-range* (butlast *avg-range*))))))
  (format t "Advertising to /joint_command.~%")
  (setf *pub* (advertise "joint_command" "nxt_msgs/JointCommand"))
  (format t "Calling border partol.~%")
  (init-border-patrol))

(defun in-grasping-range ()
  (let ((range (get-avg-range :silenced T)))
    (and range
         (< range *grasp-range*))))

(defun in-pre-grasping-range ()
  (let ((range (get-avg-range :silenced T)))
    (and range
         (<= range *pre-grasp-range*))))

(defun fetch-my-ball ()
  (let ((turn-direction 1))
    (let ((main-intents 3))
      (loop do
               (format t "Looking for ball...~%")
               (find-ball (* turn-direction 400))
               (setf turn-direction (* -1 turn-direction))
               (if (get-avg-range)
                   (setf main-intents 0)
                   (progn
                     (decf main-intents)
                     (if (< main-intents 1)
                         (format t "Could not find ball. Aborting.~%"))))
               (let ((angle 2))
                 (loop while
                       (not (in-pre-grasping-range)) do
                         (find-and-approach (* turn-direction
                                               angle)
                                            :insist T)
                         ;;(setf angle (* angle 1.5))
                         (setf turn-direction (* -1 turn-direction))
                         ;;(format t "Finished iteration. Press Enter.~%")
                         ;;(read-line)
                         (sleep 0.5)))
               (format t "In pre grasp position.~%")
               (drive-forward-slowly 0.02 :force T)
               (close-gripper)
               (sleep 1)
               (format t "Getting out of area.~%")
               (drive-forward :force T :pred
                              #'(lambda () (when (is-crossing)
                                               (setf *reached-target-area* T)
                                               T)))
               (loop while (not (is-crossing)) do
                 (sleep 0.01))
               (drive-forward :distance 0.1 :force T)
               (sleep 1)
               (open-gripper)
            while
            (and
             (> main-intents 0)
             (not (get-avg-range)))))))

(defun drive-forward-slowly (distance &key
                                        ((:pred abort-predicate) (lambda () NIL))
                                        force)
  (format t "Driving forward slowly ~a m.~%" distance)
  (let* ((rl-pos (get-avg-rl-pos))
         (distance-to-go (* pi (/ distance *wheel-circumference*) 1.7)))
    (format t "distance-to-go: ~a~%" distance-to-go)
    (loop while (and (or force (not (is-crossing)))
                     (< (- (get-avg-rl-pos) rl-pos)
                        distance-to-go)
                     (not (apply abort-predicate '()))) do
                      ;;(sleep 0.01)
                      (send-joint-commands '("l_motor_joint" "r_motor_joint") '(0.64 0.621))
                      (sleep 0.05)
                      (stop-driving))
    (if (and (not force) (is-crossing))
        (on-invalid-crossing))))

(defun drive-forward (&key distance
                        ((:pred abort-predicate) (lambda () NIL))
                        force)
  "Drive forward"
  (format t "Driving forward ~a m.~%" distance)
  (if distance
      (let* ((rl-pos (get-avg-rl-pos))
             (distance-to-go (if distance
                                 (* pi (/ distance *wheel-circumference*) 1.5))))
        (format t "distance-to-go: ~a~%" distance-to-go)
        (loop while (and (or force (not (is-crossing)))
                         (< (- (get-avg-rl-pos) rl-pos)
                            distance-to-go)
                         (not (apply abort-predicate '()))) do
                           (send-joint-commands '("l_motor_joint" "r_motor_joint") '(0.712 0.7))
                           (sleep 0.1)
                           (stop-driving))
        (if (and (not force) (is-crossing))
            (on-invalid-crossing)))
      (progn
        (send-joint-commands '("l_motor_joint" "r_motor_joint") '(0.712 0.7))
        (motor-event-trigger
         "l_motor_joint"
         (lambda (pos vel eff)
           (declare (ignore pos vel eff))
           (apply abort-predicate '()))
         (lambda (a)
           (declare (ignore a))
           (send-joint-command "l_motor_joint" 0.0)))
        (motor-event-trigger
         "r_motor_joint"
         (lambda (pos vel eff)
           (declare (ignore pos vel eff))
               (apply abort-predicate '()))
         (lambda (a)
           (declare (ignore a))
           (send-joint-command "r_motor_joint" 0.0))))))

(defun get-l-pos ()
  (elt (with-fields ((position position)) *l-motor-joint-state* position) 0))

(defun get-r-pos ()
  (elt (with-fields ((position position)) *r-motor-joint-state* position) 0))

(defun get-avg-rl-pos ()
  (let ((ret (/ (+ (get-l-pos) (get-r-pos) ) 2)))
    ;;(format t "get-avg-rl-pos: ~a~%" ret)
    ret))

(defun stop-driving ()
  "Stop driving"
  (send-joint-commands '("r_motor_joint" "l_motor_joint") '(0.0 0.0)))

(defun send-joint-command (joint effort)
  (send-joint-commands (list joint) (list effort)))

(defun send-joint-commands (joints efforts)
  (mapcar (lambda (joint effort)
    (publish-msg *pub* :name joint :effort effort)) joints efforts))

(defun close-gripper ()
  (format t "Closing gripper.~%")
  (send-joint-command "gripper" -0.6))

(defun open-gripper ()
  (format t "Opening gripper.~%")
  (send-joint-command "gripper" 0.6)
  (sleep 0.65)
  (send-joint-command "gripper" 0.0))

(defun relax-gripper ()
  (format t "Relaxing gripper.~%")
  (send-joint-command "gripper" 0.0))

(defun motor-event-trigger-callback (subscriber-key motor predicate callback msg f-callback)
  (with-fields ((name name)
                (position position)
                (velocity velocity)
                (effort effort))
      msg
      (if (string= (elt name 0) motor)
          (progn
            (if (apply predicate `(,(elt position 0) ,(elt velocity 0) ,(elt effort 0)))
                (progn
                  (format t "Triggered motor event callback.~%")
                  (unsubscribe (gethash subscriber-key *joint-state-subscriber-hash-table*))
                  (remhash subscriber-key *joint-state-subscriber-hash-table*)
                  (apply callback `(,msg)))
                (when f-callback
                  (apply f-callback '())))))))

(defun motor-event-trigger (motor predicate callback &optional f-callback)
  (let ((key (alexandria:make-gensym "joint-state")))
    (setf (gethash key *joint-state-subscriber-hash-table*)
          (subscribe "/joint_state" "sensor_msgs/JointState"
                     #'(lambda (msg) (motor-event-trigger-callback
                                      key
                                      motor
                                      predicate
                                      callback
                                      msg
                                      f-callback)))))
  nil)

(defun border-patrol ()
  (let* ((color (get-avg-color :silenced T))
         (r (when color (first color)))
         (g (when color (second color)))
         (b (when color (third color))))
    ;;(format t "r: ~a g: ~a b: ~a~%" r g b)
    (if (not (and (eq r *normal-r*) (eq g *normal-g*) (eq b *normal-b*)))
        (progn
          ;;(format t "Not on black surface?~%")
          (if (not *suspected-crossing*)
              ;; first crossing detected
              (progn
                (format t "Detected first possible crossing. ~a~%" (get-avg-color :silenced T))
                (setf *suspected-crossing* (ros-time)))
              ;; crossing has been detected before
              (progn
                (let* ((now (ros-time))
                       (time-passed (- now *suspected-crossing*)))
                  (if (> time-passed 0.25)
                      ;;(format t "Crossing!~%")
                      (format t "Crossing! ~a~%" (get-avg-color :silenced T))
                      (with-lock-held (*crossing-lock*)
                        (setf *crossing* t)))))))
        (progn
          (with-lock-held (*crossing-lock*)
            (setf *crossing* nil))
          (if *suspected-crossing*
              (progn
                (format t "Not crossing at all (~a).~%" *suspected-crossing*)
                (setf *suspected-crossing* nil)))))))

(defun init-border-patrol ()
  (format t "Subscribing to /color_sensor.~%")
  (setf *border-patrol-subscriber*
        (subscribe "/color_sensor" "nxt_msgs/Color"
                   #'(lambda (msg)
                       (with-fields ((r r)
                                     (g g)
                                     (b b)) msg
                         (with-lock-held (*avg-color-lock*)
                           (push `(,r ,g ,b) *avg-color*)
                           (setf *avg-color* (butlast *avg-color*))))
                       (border-patrol))))
  nil)

(defun order-back-border-patrol ()
  (unless (eq *border-patrol-subscriber* nil)
    (unsubscribe *border-patrol-subscriber*)
    (format t "Unsubscribed color sensor.~%")))

(defun on-invalid-crossing ()
  (format t "Intervening against invalid crossing.~%")
  (stop-driving)
  (turn 180))

(defun is-crossing ()
  (with-lock-held (*crossing-lock*)
    *crossing*))


;(defun set-velocity (motor value)
;  (motor-event-trigger motor (lambda (pos vel eff)
;                               ())))

(defun turn (degree &optional (abort-predicate (lambda () NIL)))
  (format t "Turning ~a degree.~%" degree)
  (setf degree (* degree 0.0174532925))
  (let* ((l (* 0.67 (signum degree)))
         (r (* 0.65 (- (signum degree))))
         (l-pos (get-l-pos))
         (r-pos (get-r-pos))
         (distance-to-go (abs (* 3.00 degree))))
    (loop while (and
                 (<= (abs (- (get-l-pos) l-pos))
                     distance-to-go)
                 (not (apply abort-predicate '()))) do
                   (send-joint-commands '("l_motor_joint" "r_motor_joint") `(,l ,r))
                   (sleep 0.0001)
                   (stop-driving))))

(defun find-ball (max-degree &key insist)
  (let ((first-intent T))
    (format t "Finding ball. max-degree ~a~%" max-degree)
    (let ((turn-direction 1)
          (angle max-degree))
      (loop do
        (format t "Still finding ball.~%")
        (format t "angle: ~a~%" angle)
        (turn (* turn-direction angle)
              #'(lambda () (get-avg-range :silenced T)))
        (if first-intent
            (progn
              (setf first-intent nil)
              (setf angle 1.0))
            (setf angle (* angle 2)))
        (format t "New angle: ~a~%" angle)
        (setf turn-direction (* -1 turn-direction))
        (sleep 1)
            while (and
                   insist
                   (not (get-avg-range)))))
    (format t "Finished finding-ball.~%")
  (format t "Obstacle in Range: ~a.~%" (get-avg-range :silenced T))))

(defun get-avg-range (&key silenced)
  (let ((ret (with-lock-held (*avg-range-lock*)
               (let ((range *avg-range*))
                 (if (> (length (remove-if #'(lambda (e)  e) range)) 1)
                     nil
                     (let ((values (remove-if #'(lambda (e) (not e)) range)))
                       (/ (reduce #'+ values) (length values))))))))
    (unless silenced
      (format t "get-avg-range: ~a~%" ret))
    ret))

(defun get-colors ()
  `(,'(0.0 0.0 0.0)
    ,'(0.0 0.0 1.0)
    ,'(0.0 1.0 0.0)
    ,'(0.0 1.0 1.0)
    ,'(1.0 0.0 0.0)
    ,'(1.0 0.0 1.0)
    ,'(1.0 1.0 0.0)
    ,'(1.0 1.0 1.0)))

(defun create-color-occurences-hash ()
  (let ((hash (make-hash-table :test 'equalp)))
    (loop for c in (get-colors) do
      (setf (gethash c hash) 0))    
    hash))

(defun get-highest-color-count (hash)
  (let ((highest 0)
        (ret nil))
    (loop for c in (get-colors) do
      (let ((count (gethash c hash)))
        ;;(format t "color: ~a count: ~a highest: ~a~%" c count highest)
        (when (> count highest)
          (progn
            (setf highest count)
            (setf ret c)))))
    ret))

(defun get-avg-color (&key silenced)
  (let* ((color-occurences (create-color-occurences-hash))
         (ret (progn
                (with-lock-held (*avg-color-lock*)
                (let ((color *avg-color*))
                  (mapcar
                   #'(lambda (e)
                       (when (gethash e color-occurences)
                         (setf (gethash e color-occurences)
                               (incf (gethash e color-occurences)))))
                   color)))
              (get-highest-color-count color-occurences))))
    (unless silenced
      (format t "get-avg-color: ~a~%" ret))
    ret))

(defun approach ()
  (format t "Approaching...~%")
  (let ((last-known-range (get-avg-range)))
    (if last-known-range
      (let ((distance-to-go (- last-known-range *pre-grasp-range*)))
        (format t "distance-to-go: ~a m~%" distance-to-go)
        (if (< distance-to-go 0.10)
            (drive-forward-slowly distance-to-go
                           :pred #'(lambda () (not (get-avg-range))))
            (drive-forward :distance distance-to-go
                           :pred #'(lambda () (not (get-avg-range))))))
      (format t "Lost object out of eyes.~%")))
  (format t "Approched as much as I could.~%"))

(defun find-and-approach (max-degree &key insist)
  (format t "Finding and approaching.~%")
  (find-ball max-degree :insist insist)
  (sleep 1)
  (if (get-avg-range)
        (approach)
        (progn
          (format t "Moved a little bit too far. Correcting.~%")
          ;;(find-ball (* (- (signum max-degree)) 2))
          (approach))))