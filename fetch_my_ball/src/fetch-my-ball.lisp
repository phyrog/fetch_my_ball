(in-package :fetch-my-ball)

(defvar *wheelbase* 0.115)
(defvar *wheel-radius* 0.027)
(defvar *wheel-circumference* (* 2 pi *wheel-radius*))
(defvar *pub*)
(defvar *border-patrol-subscriber*)
(defvar *joint-state-subscriber-hash-table* (make-hash-table))
(defvar *suspected-crossing* nil)
(defvar *l-motor-joint-state*)
(defvar *r-motor-joint-state*)
(defvar *gripper-state*)
(defvar *avg-range-lock* (make-lock "avg-range-lock"))
(defvar *avg-range* '(nil nil nil))
(defvar *max-grasping-range* 0.06)

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
                                    (push (if (< range 0.5)
                                              range
                                              nil)
                                          *avg-range*)
                                    (butlast *avg-range*)))))
  (format t "Advertising to /joint_command.~%")
  (setf *pub* (advertise "joint_command" "nxt_msgs/JointCommand")))

(defun fetch-my-ball ()
  )

(defun drive-forward (&optional distance (abort-predicate (lambda () NIL)))
  "Drive forward"
  (format t "Driving forward ~a m.~%" distance)
  (let* ((l-joint-state (elt (with-fields ((position position)) *l-motor-joint-state* position) 0))
         (r-joint-state (elt (with-fields ((position position)) *r-motor-joint-state* position) 0))
         (distance-to-go (if distance
                             (* pi (/ distance *wheel-circumference*) 1.7))))
    (format t "distance-to-go: ~a~%" distance-to-go)
    (send-joint-commands '("l_motor_joint" "r_motor_joint") '(0.712 0.7))
    (motor-event-trigger
     "l_motor_joint"
     (lambda (pos vel eff)
       (declare (ignore vel eff))
       (or (if distance
               (>= (- pos l-joint-state)
                   distance-to-go))
           (apply abort-predicate '())))
     (lambda (a)
       (declare (ignore a))
       (send-joint-command "l_motor_joint" 0.0)))
    (motor-event-trigger
     "r_motor_joint"
     (lambda (pos vel eff)
       (declare (ignore vel eff))
       (or (if distance
               (>= (- pos r-joint-state)
                   distance-to-go))
           (apply abort-predicate '())))
     (lambda (a)
       (declare (ignore a))
       (send-joint-command "r_motor_joint" 0.0)))))

(defun get-avg-rl-joint-state ()
  (let* ((l-joint-state (elt (with-fields ((position position)) *l-motor-joint-state* position) 0))
         (r-joint-state (elt (with-fields ((position position)) *r-motor-joint-state* position) 0)))
    (/ (+ l-joint-state r-joint-state) 2)))

(defun drive-forward-slowly (distance &optional (abort-predicate (lambda () NIL)))
  (format t "Driving forward slowly ~a m.~%" distance)
  (let* ((rl-joint-state (get-avg-rl-joint-state))
         (distance-to-go (* pi (/ distance *wheel-circumference*) 1.7)))
    (format t "distance-to-go: ~a~%" distance-to-go)
    (loop while (and (< (- (get-avg-rl-joint-state) rl-joint-state)
                        distance-to-go)
                    (not (apply abort-predicate '()))) do
                      ;;(sleep 0.01)
                      (send-joint-commands '("l_motor_joint" "r_motor_joint") '(0.64 0.621))
                      (sleep 0.05)
                      (stop-driving))))

(defun stop-driving ()
  "Stop driving"
  (send-joint-commands '("r_motor_joint" "l_motor_joint") '(0.0 0.0)))

(defun send-joint-command (joint effort)
  (send-joint-commands (list joint) (list effort)))

(defun send-joint-commands (joints efforts)
  (mapcar (lambda (joint effort)
    (publish-msg *pub* :name joint :effort effort)) joints efforts))

(defun close-gripper ()
  (send-joint-command "gripper" -0.6))

(defun open-gripper ()
  (send-joint-command "gripper" 0.6)
  (sleep 0.5)
  (send-joint-command "gripper" 0.0))

(defun relax-gripper ()
  (send-joint-command "gripper" 0.0))

(defun drive-angle (radius angle)
  (let* ((w-a-radius (- radius (/ *wheelbase* 2.0)))
         (w-b-radius (+ radius (/ *wheelbase* 2.0)))
         (length-a (* (/ angle 180.0) (* PI w-a-radius)))
         (length-b (* (/ angle 180.0) (* PI w-b-radius)))
         (ratio-ab (if (< (/ length-a length-b) 1.0) 
                     (/ length-a length-b)
                     (/ length-b length-a)))
         (effort-a (+ 0.6 (if (< ratio-ab 1.0) 0.2 (* 0.2 ratio-ab))))
         (effort-b (+ 0.6 (if (> ratio-ab 1.0) 0.2 (* 0.2 ratio-ab)))))
    (format t "~a ~a" effort-a effort-b)
    (send-joint-commands '("l_motor_joint" "r_motor_joint") `(,effort-a ,effort-b))))

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

(defun border-patrol (msg)
  (with-fields ((r r)
                (g g)
                (b b))
      msg
    ;;(format t "r: ~a g: ~a b: ~a~%" r g b)
    (if (not (and (= r 0) (= g 0) (= b 0)))
        (progn
          ;;(format t "Not on black surface?~%")
          (if (not *suspected-crossing*)
              ;; first crossing detected
              (progn
                (format t "Detected first possible crossing.~%")
                (setf *suspected-crossing* (ros-time)))
              ;; crossing has been detected before
              (progn
                (let* ((now (ros-time))
                       (time-passed (- now *suspected-crossing*)))
                  (if (> time-passed 0.5)
                      (format t "Crossing!~%"))))))
        (if *suspected-crossing*
            (progn
              (format t "~a Was no crossing at all.~%" *suspected-crossing*)
              (setf *suspected-crossing* nil))))))


(defun init-border-patrol ()
  (setf *border-patrol-subscriber*
        (subscribe "/color_sensor" "nxt_msgs/Color" #'border-patrol))
  (format t "Subscribed to color sensor.~%"))

(defun order-back-border-patrol ()
  (unless (eq *border-patrol-subscriber* nil)
    (unsubscribe *border-patrol-subscriber*)
    (format t "Unsubscribed color sensor.~%")))

(defun on-invalid-crossing ()
  (stop-driving)
  (turn 180))


;(defun set-velocity (motor value)
;  (motor-event-trigger motor (lambda (pos vel eff)
;                               ())))

(defun turn (degree &optional (abort-predicate (lambda () NIL)))
  (format t "Turning ~a degree.~%" degree)
  (setf degree (* degree 0.0174532925))
  (let* ((l (* 0.7125 (signum degree)))
         (r (* 0.7 (- (signum degree))))
         (l-joint-state (with-fields ((position position)) *l-motor-joint-state* position))
         (r-joint-state (with-fields ((position position)) *r-motor-joint-state* position)))
    (send-joint-commands '("l_motor_joint" "r_motor_joint") `(,l ,r))
    (motor-event-trigger
     "l_motor_joint"
     (lambda (pos vel eff)
       (declare (ignore vel eff))
       (or (>= (abs (- pos (elt l-joint-state 0)))
               (abs (* 1.6 degree)))
           (apply abort-predicate '())))
     (lambda (a)
       (declare (ignore a))
       (send-joint-command "l_motor_joint" 0.0)))
    (motor-event-trigger
     "r_motor_joint"
     (lambda (pos vel eff)
       (declare (ignore vel eff))
       (or (>= (abs (- pos (elt r-joint-state 0)))
               (abs (* 1.6 degree)))
           (apply abort-predicate '())))
     (lambda (a)
       (declare (ignore a))
       (send-joint-command "r_motor_joint" 0.0)))))

(defun find-ball (max-degree)
  (format t "Finding ball.~%") 
  (turn max-degree #'(lambda ()
                       (with-lock-held (*avg-range-lock*)
                         (let ((one (first *avg-range*))
                               (two (second *avg-range*))
                                   (three (third *avg-range*)))
                           (if (and one two three)
                               (progn
                                 (format t "Obstacle in Range: ~a, ~a, ~a.~%" one two three)
                                 T)))))))

(defun get-avg-range ()
  (with-lock-held (*avg-range-lock*)
    (let ((one (first *avg-range*))
          (two (second *avg-range*))
          (three (third *avg-range*)))
      (if (and one two three)
          (/ (+ one two three) 3)
          nil))))

(defun approach-until-max-range ()
  (let ((last-known-range (get-avg-range)))
    (drive-forward (- last-known-range *max-grasping-range*)
                   #'(lambda () (not (get-avg-range))))))