(in-package :fetch-my-ball)

(defvar *wheelbase* 0.115 "Radabstand")
(defvar *wheel-radius* 0.027 "Radradius")
(defvar *wheel-circumference* (* 2 pi *wheel-radius*) "Radumfang")
(defvar *pub* "Publisher für joint-Kommandos")
(defvar *border-patrol-subscriber* "Subscriber für Grenzkontrolle.")
(defvar *joint-state-subscriber-hash-table* (make-hash-table)
  "Hash Table für Subscriber auf /joint_state.
Wird für die Funktion motor-event-trigger benötigt.")
(defvar *normal-r* 0.0 "Rot-Anteil für die Farbe des Untergrunds
innerhalb des Rechtecks, in dem Robby sich befindet.")
(defvar *normal-g* 0.0 "Grün-Anteil für die Farbe des Untergrunds
innerhalb des Rechtecks, in dem Robby sich befindet.")
(defvar *normal-b* 0.0 "Blau-Anteil für die Farbe des Untergrunds
innerhalb des Rechtecks, in dem Robby sich befindet.")
(defvar *avg-color-lock* (make-lock)
  "Lock für die Liste der letzten Farbwerte.")
(defvar *avg-color* '(nil nil nil nil nil nil)
  "Liste der letzten Farbwerte des RGB-Sensors.")
(defvar *suspected-crossing* nil
  "Gibt an, ob ein Überfahren der Grenze vermutet wird.")
(defvar *crossing* nil
  "Gibt an, ob die Grenze gerade überfahren wird.")
(defvar *crossing-lock* (make-lock)
  "Lock für die Variable *crossing*")
(defvar *l-motor-joint-state*
  "Joint State des linken Antriebmotors")
(defvar *r-motor-joint-state*
  "Joint State des rechten Antriebmotors")
(defvar *gripper-state*
  "Joint State des Motors des Greifers.")
(defvar *avg-range-lock* (make-lock)
  "Lock für die Liste der letzten Abstandsmessungen.")
(defvar *avg-range* '(nil nil nil nil nil nil)
  "Liste der letzten Abstandsmessungen des Ultraschallsensors.")
(defvar *pre-grasp-range* 0.075
  "Maximaler Abstand zum Ball für die Prä-Greifposition.")
(defvar *grasp-range* 0.02
  "Maximaler Abstand zum Ball zum Greifen.")
(defvar *reached-target-area* nil
  "Gibt an, ob Robby nach dem Greifen die Grenze überschritten hat.
Wird beim Ablegen des Balls benötigt.")
(defvar *achieved-goal* nil
  "Gibt an, ob das Ziel erreicht wurde.")

(defun init-driver ()
  "Initialisert alle benötigten Variablen und Vorgänge,
subscribed auf alle benötigten Topics mit den dazugehörigen Callbacks."
  (format t "Starting up ros.~%")
  (roslisp-utilities:startup-ros)
  (format t "Subscribing to /joint_state.~%")
  (subscribe "joint_state" "sensor_msgs/JointState" (lambda (val)
    (with-fields ((name name)) val
      (cond ((string= (elt name 0) "l_motor_joint") (setf *l-motor-joint-state* val))
            ((string= (elt name 0) "r_motor_joint") (setf *r-motor-joint-state* val))
            ((string= (elt name 0) "gripper") (setf *gripper-state* val))))))
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
  "Gibt an, ob Robby sich nah genug am Ball befindet, um zu greifen."
  (let ((range (get-avg-range :silenced T)))
    (and range
         (< range *grasp-range*))))

(defun in-pre-grasping-range ()
  "Gibt an, ob Robby sich in der Prä-Greifposition befindet."
  (let ((range (get-avg-range :silenced T)))
    (and range
         (<= range *pre-grasp-range*))))

(defun fetch-my-ball ()
  "Startet die Suche nach dem Ball. Robby dreht sich zuerst ca. 360° im Uhrzeigersinn.
Findet er dabei den Ball (bzw. einen Gegenstand) per Ultraschallsensor, hält er an und
fährt anschließend so lange geradeaus, bis der Ultraschallsensor entweder den Gegenstand
'verliert' oder Robby sich in der Prä-Greifposition befindet.

Findet Robby bei der kompletten Drehung jedoch keinen Gegenstand, so dreht er sich ca.
360° gegen den Uhrzeigersinn. Diesen vorgang wiederholt er insgesamt maximal drei mal.

Befindet Robby sich in der Prä-Greifposition, fährt er ca. 2cm geradeaus und schließt
anschließend den Greifarm. Dann fährt er so lange geradeaus, bis er die Grenze überschreitet,
fährt anschließend weitere 10cm und öffnet den Greifarm."
  (setf *achieved-goal* nil)
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
               (setf *achieved-goal* T)
            while
            (and
             (> main-intents 0)
             (not (get-avg-range))
             (not *achieved-goal*))))))

(defun drive-forward-slowly (distance &key
                                        ((:pred abort-predicate) (lambda () NIL))
                                        force)
  "Lässt Robby langsam vorwärts fahren.
`distance' Strecke in Metern, die Robby fahren soll.
`pred' Ein Prädikat, welches das Vorwärtsfahren abbricht, wenn es `T' zurückgibt.
`force' Wenn `T' wird ein Überfahren der Grenze ignoriert. Ansonsten hält Robby beim
Überfahren der Grenze an und dreht sich um ca. 180°."
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
  "Lässt Robby vorwärts fahren.
`distance' Strecke in Metern, die Robby fahren soll. Falls `nil' hält Robby nicht an
und diese Funktion blockiert nicht.
`pred' Ein Prädikat, welches das Vorwärtsfahren abbricht, wenn es `T' zurückgibt.
`force' Wenn `T' wird ein Überfahren der Grenze ignoriert. Ansonsten hält Robby beim
Überfahren der Grenze an und dreht sich um ca. 180°."
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
  "Gibt die Position des linken Antriebmotors zurück."
  (elt (with-fields ((position position)) *l-motor-joint-state* position) 0))

(defun get-r-pos ()
  "Gibt die Position des rechten Antriebmotors zurück."
  (elt (with-fields ((position position)) *r-motor-joint-state* position) 0))

(defun get-avg-rl-pos ()
  "Gibt den Durschnitt der Positionen des linken und des rechten Antriebmotors zurück."
  (let ((ret (/ (+ (get-l-pos) (get-r-pos) ) 2)))
    ;;(format t "get-avg-rl-pos: ~a~%" ret)
    ret))

(defun stop-driving ()
  "Stoppt alle Antriebsmotoren."
  (send-joint-commands '("r_motor_joint" "l_motor_joint") '(0.0 0.0)))

(defun send-joint-command (joint effort)
  "Sendet ein Joint Command an das /joint_command-Topic.
`joint' Name des Joints (z. B. 'r_motor_joint ')
`effort' Leistung, die der Motor aufbringen soll."
  (send-joint-commands (list joint) (list effort)))

(defun send-joint-commands (joints efforts)
  "Sendet mehrere Joint Commands an das /joint_command-Topic.
`joint' Liste von Namen von Joints
`effort' Liste von Leistungen, die der Motor aufbringen soll.
Das n-te Element der joint-Liste wird mit dem n-ten Element
der effort-Liste verwendet."
  (mapcar (lambda (joint effort)
    (publish-msg *pub* :name joint :effort effort)) joints efforts))

(defun close-gripper ()
  "Schließt den Greifarm und übt nach dem Schließen weiterhin Druck aus."
  (format t "Closing gripper.~%")
  (send-joint-command "gripper" -0.6))

(defun open-gripper ()
  "Öffnet den Greifarm."
  (format t "Opening gripper.~%")
  (send-joint-command "gripper" 0.6)
  (sleep 0.65)
  (send-joint-command "gripper" 0.0))

(defun relax-gripper ()
  "Setzt die Kraft, die der Greifarm ausübt auf Null."
  (format t "Relaxing gripper.~%")
  (send-joint-command "gripper" 0.0))

(defun motor-event-trigger-callback (subscriber-key motor predicate callback msg f-callback)
  "Callback für Subscriber aus 'motor-event-trigger'."
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
  "Subscribed zur Überwachung eines Motors auf das /joint_state-Topic.
Wird ein bestimmter Zustand erreicht, wird eine Aktion ausgeführt.
`motor' Der name des Motors, der Überwacht werden soll.
`predicate' Wenn dieses Prädikat erfüllt wird, wird `callback' ausgeführt.
`callback' Wird ausgeführt, wenn `predicate' erfüllt wird. Die Überwachung des
Motors wird beendet.
`f-callback' Wird ausgeführt, wenn `predicate' nicht erfüllt wird."
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
  "Grenzkontrolle. Überprüft, ob Robby sich auf der Grenze befindet.
Wird anhand des RGB-Sensordś eine potentielle Grenzüberfahrt registriert,
wird nach 0,25 Sekunden erneut überprüft, ob der RGB-Sensor immernoch eine
Grenzüberfahrt anzeigt. Erst wenn das der Fall ist, wird angenommen, dass
tatsächlich eine Grenzüberfahrt passiert ist."
  (let* ((color (get-avg-color :silenced T))
         (r (when color (first color)))
         (g (when color (second color)))
         (b (when color (third color))))
    ;;(format t "r: ~a g: ~a b: ~a~%" r g b)
    (if (not (and (eq r *normal-r*) (eq g *normal-g*) (eq b *normal-b*)))
        (progn
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
  "Initialisiert die Grenzkontrolle."
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
  "Beendet die Grenzkontrolle."
  (unless (eq *border-patrol-subscriber* nil)
    (unsubscribe *border-patrol-subscriber*)
    (format t "Unsubscribed color sensor.~%")))

(defun on-invalid-crossing ()
  "Funktion, welche ausgeführt wird,
sobald beim Vorwährtsfahren eine Grenzüberfahrt registriert wurde.
Robby hält an und dreht sich um ca 180°."
  (format t "Intervening against invalid crossing.~%")
  (stop-driving)
  (turn 180))

(defun is-crossing ()
  "Gibt den Wert der globalen Variable `*crossing*' zurück."
  (with-lock-held (*crossing-lock*)
    *crossing*))

(defun turn (degree &optional (abort-predicate (lambda () NIL)))
  "Dreht Robby auf der Stelle um ca. `degree' grad. Bei positiven
Werten dreht er sich im Uhrzeigersinn, bei negative Werten entgegen
dem Uhrzeigersinn.
Bricht ab, sobald `abort-predicate' erfüllt ist."
  (format t "Turning ~a degree.~%" degree)
  (setf degree (* degree 0.0174532925))
  (let* ((l (* 0.67 (signum degree)))
         (r (* 0.65 (- (signum degree))))
         (l-pos (get-l-pos))
         (distance-to-go (abs (* 3.00 degree))))
    (loop while (and
                 (<= (abs (- (get-l-pos) l-pos))
                     distance-to-go)
                 (not (apply abort-predicate '()))) do
                   (send-joint-commands '("l_motor_joint" "r_motor_joint") `(,l ,r))
                   (sleep 0.0001)
                   (stop-driving))))

(defun find-ball (max-degree &key insist)
  "Sucht den Ball in einem Winkel von `max-degree' und hält beim registrieren eines
Gegenstands an.
Wenn `insist' nicht `nil' ist, sucht Robby weiter, nachdem er den Ball gefunden hat,
sich aber zu weit bewegt hat und der Ultraschallsensor somit keinen Ball mehr registriert.
Dabei dreht er sich 1° in eine richtung, dann 2° in die andere richtung, usw. (Gradzahl
verdoppelt sich immer, Richtung wechselt sich ab), bis maximal 400°."
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
                   (not (get-avg-range))
                   (<= angle 400))))
    (format t "Finished finding-ball.~%")
  (format t "Obstacle in Range: ~a.~%" (get-avg-range :silenced T))))

(defun get-avg-range (&key silenced)
  "Gibt den durchschnittlichen Wert der letzten sieben Werte des Ultraschallsensors zurück.
Wenn mehr als zwei dieser Werte außerhalb der maximalen Reichweite liegen, wird `nil' zurückgegeben.
`silenced' Wenn `nil' wird der Wert in die Ausgabe geschrieben, sonst nicht."
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
  "Gibt eine Liste aller möglichen Farbkombinationen zurück."
  `(,'(0.0 0.0 0.0)
    ,'(0.0 0.0 1.0)
    ,'(0.0 1.0 0.0)
    ,'(0.0 1.0 1.0)
    ,'(1.0 0.0 0.0)
    ,'(1.0 0.0 1.0)
    ,'(1.0 1.0 0.0)
    ,'(1.0 1.0 1.0)))

(defun create-color-occurences-hash ()
  "Erstellt ein Hashtabelle, deren Schlüssel aus allen möglichen Farkombinationen bestehen.
Die Werte werden alle auf `0' gesetzt."
  (let ((hash (make-hash-table :test 'equalp)))
    (loop for c in (get-colors) do
      (setf (gethash c hash) 0))    
    hash))

(defun get-highest-color-count (hash)
  "Gibt die Farbkombination zurück, die in der übergebenen Hashtabelle
`hash' den höchsten Wert besitzt."
  (let ((highest 0)
        (ret nil))
    (loop for c in (get-colors) do
      (let ((count (gethash c hash)))
        (when (> count highest)
          (progn
            (setf highest count)
            (setf ret c)))))
    ret))

(defun get-avg-color (&key silenced)
  "Gibt aus den letzten sieben Farbwerten des RGB-Sensors den zurück, der am häufigsten vorkommt.
`silenced' Wenn `nil' wird der Wert in die Ausgabe geschrieben, sonst nicht."
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
  "Lässt Robby solange geradeausfahren, wie der Ultraschallsensor einen Gegenstand
innerhalb der maximalen Reichweite erkennt oder Robby die Grenze überfährt."
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
  "Sucht den Ball in einem Winkel von `max-degree'. Registriert der Ultraschallsensor dabei
einen Gegenstand, fährt Robby solange geradeaus, wie der Ultraschallsensor einen Gegenstand
innerhalb der maximalen Reichweite erkennt oder Robby die Grenze überfährt.
Wenn `insist' nicht `nil' ist, sucht Robby weiter, nachdem er den Ball gefunden hat,
sich aber zu weit bewegt hat und der Ultraschallsensor somit keinen Ball mehr registriert."
  (format t "Finding and approaching.~%")
  (find-ball max-degree :insist insist)
  (sleep 1)
  (approach))