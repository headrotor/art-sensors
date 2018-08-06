from stepper import Stepper

#stepper variables
#[stepPin, directionPin, enablePin]
testStepper = Stepper([23, 24, 22])

#test stepper
testStepper.step(1600, "right"); #steps, dir, speed, stayOn
