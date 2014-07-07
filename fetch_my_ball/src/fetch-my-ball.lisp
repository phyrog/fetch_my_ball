(in-package :fetch-my-ball)

(setf *wheelbase* 11.5)

(defun init-driver ()
  (roslisp-utilities:startup-ros)
  (setf *pub* (advertise "joint_command" "nxt_msgs/JointCommand")))

(defun drive-forward ()
  "Drive forward"
  (send-joint-commands '("r_motor_joint" "l_motor_joint") '(0.7 0.7)))

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

(defun turn (direction)
  (send-joint-command "l_motor_joint" (* direction 0.8))
  (send-joint-command "r_motor_joint" (* -1 (* direction 0.8))))

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
