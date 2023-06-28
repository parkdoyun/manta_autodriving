
"use strict";

let SensorPosControl = require('./SensorPosControl.js');
let ScenarioLoad = require('./ScenarioLoad.js');
let SyncModeCmd = require('./SyncModeCmd.js');
let ObjectStatusList = require('./ObjectStatusList.js');
let MoraiSrvResponse = require('./MoraiSrvResponse.js');
let PRCtrlCmd = require('./PRCtrlCmd.js');
let WaitForTickResponse = require('./WaitForTickResponse.js');
let GetTrafficLightStatus = require('./GetTrafficLightStatus.js');
let EventInfo = require('./EventInfo.js');
let SyncModeAddObject = require('./SyncModeAddObject.js');
let SaveSensorData = require('./SaveSensorData.js');
let RadarDetections = require('./RadarDetections.js');
let SyncModeCtrlCmd = require('./SyncModeCtrlCmd.js');
let SyncModeInfo = require('./SyncModeInfo.js');
let RadarDetection = require('./RadarDetection.js');
let CollisionData = require('./CollisionData.js');
let SyncModeCmdResponse = require('./SyncModeCmdResponse.js');
let MapSpec = require('./MapSpec.js');
let VehicleSpec = require('./VehicleSpec.js');
let VehicleSpecIndex = require('./VehicleSpecIndex.js');
let IntersectionControl = require('./IntersectionControl.js');
let ObjectStatus = require('./ObjectStatus.js');
let SyncModeSetGear = require('./SyncModeSetGear.js');
let ERP42Info = require('./ERP42Info.js');
let SetTrafficLight = require('./SetTrafficLight.js');
let MoraiTLInfo = require('./MoraiTLInfo.js');
let PRStatus = require('./PRStatus.js');
let WaitForTick = require('./WaitForTick.js');
let GhostMessage = require('./GhostMessage.js');
let MapSpecIndex = require('./MapSpecIndex.js');
let ReplayInfo = require('./ReplayInfo.js');
let CtrlCmd = require('./CtrlCmd.js');
let SyncModeResultResponse = require('./SyncModeResultResponse.js');
let VehicleCollision = require('./VehicleCollision.js');
let NpcGhostCmd = require('./NpcGhostCmd.js');
let TrafficLight = require('./TrafficLight.js');
let IntersectionStatus = require('./IntersectionStatus.js');
let EgoVehicleStatus = require('./EgoVehicleStatus.js');
let DdCtrlCmd = require('./DdCtrlCmd.js');
let SyncModeScenarioLoad = require('./SyncModeScenarioLoad.js');
let NpcGhostInfo = require('./NpcGhostInfo.js');
let SyncModeRemoveObject = require('./SyncModeRemoveObject.js');
let Lamps = require('./Lamps.js');
let GPSMessage = require('./GPSMessage.js');
let IntscnTL = require('./IntscnTL.js');
let EgoDdVehicleStatus = require('./EgoDdVehicleStatus.js');
let PREvent = require('./PREvent.js');
let MoraiSimProcHandle = require('./MoraiSimProcHandle.js');
let VehicleCollisionData = require('./VehicleCollisionData.js');
let MultiEgoSetting = require('./MultiEgoSetting.js');
let MoraiSimProcStatus = require('./MoraiSimProcStatus.js');
let MoraiTLIndex = require('./MoraiTLIndex.js');

module.exports = {
  SensorPosControl: SensorPosControl,
  ScenarioLoad: ScenarioLoad,
  SyncModeCmd: SyncModeCmd,
  ObjectStatusList: ObjectStatusList,
  MoraiSrvResponse: MoraiSrvResponse,
  PRCtrlCmd: PRCtrlCmd,
  WaitForTickResponse: WaitForTickResponse,
  GetTrafficLightStatus: GetTrafficLightStatus,
  EventInfo: EventInfo,
  SyncModeAddObject: SyncModeAddObject,
  SaveSensorData: SaveSensorData,
  RadarDetections: RadarDetections,
  SyncModeCtrlCmd: SyncModeCtrlCmd,
  SyncModeInfo: SyncModeInfo,
  RadarDetection: RadarDetection,
  CollisionData: CollisionData,
  SyncModeCmdResponse: SyncModeCmdResponse,
  MapSpec: MapSpec,
  VehicleSpec: VehicleSpec,
  VehicleSpecIndex: VehicleSpecIndex,
  IntersectionControl: IntersectionControl,
  ObjectStatus: ObjectStatus,
  SyncModeSetGear: SyncModeSetGear,
  ERP42Info: ERP42Info,
  SetTrafficLight: SetTrafficLight,
  MoraiTLInfo: MoraiTLInfo,
  PRStatus: PRStatus,
  WaitForTick: WaitForTick,
  GhostMessage: GhostMessage,
  MapSpecIndex: MapSpecIndex,
  ReplayInfo: ReplayInfo,
  CtrlCmd: CtrlCmd,
  SyncModeResultResponse: SyncModeResultResponse,
  VehicleCollision: VehicleCollision,
  NpcGhostCmd: NpcGhostCmd,
  TrafficLight: TrafficLight,
  IntersectionStatus: IntersectionStatus,
  EgoVehicleStatus: EgoVehicleStatus,
  DdCtrlCmd: DdCtrlCmd,
  SyncModeScenarioLoad: SyncModeScenarioLoad,
  NpcGhostInfo: NpcGhostInfo,
  SyncModeRemoveObject: SyncModeRemoveObject,
  Lamps: Lamps,
  GPSMessage: GPSMessage,
  IntscnTL: IntscnTL,
  EgoDdVehicleStatus: EgoDdVehicleStatus,
  PREvent: PREvent,
  MoraiSimProcHandle: MoraiSimProcHandle,
  VehicleCollisionData: VehicleCollisionData,
  MultiEgoSetting: MultiEgoSetting,
  MoraiSimProcStatus: MoraiSimProcStatus,
  MoraiTLIndex: MoraiTLIndex,
};
