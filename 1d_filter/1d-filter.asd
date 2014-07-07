(defsystem 1d-filter
  :author "Tom Gehrke <tom.gehrke@uni-bremen.de>"

  :depends-on (roslisp
               roslisp-utilities
               nxt_msgs-msg
               sensor_msgs-msg
               bordeaux-threads)
  :components
  ((:module "src"
    :components
      ((:file "package")
       (:file "1d-filter" :depends-on ("package"))))))
