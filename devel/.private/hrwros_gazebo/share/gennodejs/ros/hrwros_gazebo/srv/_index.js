
"use strict";

let AGVControl = require('./AGVControl.js')
let SubmitTray = require('./SubmitTray.js')
let GetMaterialLocations = require('./GetMaterialLocations.js')
let VacuumGripperControl = require('./VacuumGripperControl.js')
let PopulationControl = require('./PopulationControl.js')
let ConveyorBeltControl = require('./ConveyorBeltControl.js')

module.exports = {
  AGVControl: AGVControl,
  SubmitTray: SubmitTray,
  GetMaterialLocations: GetMaterialLocations,
  VacuumGripperControl: VacuumGripperControl,
  PopulationControl: PopulationControl,
  ConveyorBeltControl: ConveyorBeltControl,
};
