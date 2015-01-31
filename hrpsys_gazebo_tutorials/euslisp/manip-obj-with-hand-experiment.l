
(defun test1
  ()

  (init :model-name "small_box" :model-num 6)

  (init-variable)

  (manip :manip-input (gen-strategy-from-force #f(40 0 0) #f(40 0 0) :num 100))

  (manip :manip-input (gen-strategy-from-force #f(0 0 2) #f(0 0 2) :num 100))
  (manip :manip-input (gen-strategy-from-force #f(0 0 8) #f(0 0 8) :num 100))

  (manip :manip-input (gen-strategy-from-force #f(0 0 -2) #f(0 0 -2) :num 100))
  (manip :manip-input (gen-strategy-from-force #f(0 0 -8) #f(0 0 -8) :num 100))

  (analyze :class-num 4)

  (print-result)

  ;;(manip-all-class-data)
  )

(defun test2
  ()

  (init :model-name "large_box" :model-num 6)

  (init-variable)

  (manip :manip-input (gen-strategy-from-force #f(40 0 0) #f(40 0 0) :num 50))

  (manip :manip-input (gen-strategy-from-force #f(0 0 10) #f(0 0 10) :num 50))
  (manip :manip-input (gen-strategy-from-force #f(0 0 50) #f(0 0 50) :num 50))

  (manip :manip-input (gen-strategy-from-force #f(0 0 -10) #f(0 0 -10) :num 50))
  (manip :manip-input (gen-strategy-from-force #f(0 0 -50) #f(0 0 -50) :num 50))

  (analyze :class-num 4)

  (print-result)

  ;;(manip-all-class-data)
  )


