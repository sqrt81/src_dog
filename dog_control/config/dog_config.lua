require("physics")
require("model")
require("motor")
require("control")
require("estimator")

options = {
  physics = PHYSICS,
  model = MODEL,
  motor = MOTOR_CONFIG,
  motor_name = MOTOR_NAME,
  control = CONTROL,
  estimator = ESTIMATOR,
}

return options

