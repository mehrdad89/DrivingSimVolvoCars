# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: osi3/osi_object.proto
"""Generated protocol buffer code."""
from google.protobuf.internal import builder as _builder
from google.protobuf import descriptor as _descriptor
from google.protobuf import descriptor_pool as _descriptor_pool
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


from osi3 import osi_common_pb2 as osi3_dot_osi__common__pb2


DESCRIPTOR = _descriptor_pool.Default().AddSerializedFile(b'\n\x15osi3/osi_object.proto\x12\x04osi3\x1a\x15osi3/osi_common.proto\"\xfc\x0e\n\x10StationaryObject\x12\x1c\n\x02id\x18\x01 \x01(\x0b\x32\x10.osi3.Identifier\x12\"\n\x04\x62\x61se\x18\x02 \x01(\x0b\x32\x14.osi3.BaseStationary\x12=\n\x0e\x63lassification\x18\x03 \x01(\x0b\x32%.osi3.StationaryObject.Classification\x12\x17\n\x0fmodel_reference\x18\x04 \x01(\t\x12\x31\n\x10source_reference\x18\x05 \x03(\x0b\x32\x17.osi3.ExternalReference\x12\x31\n\x11\x63olor_description\x18\x06 \x01(\x0b\x32\x16.osi3.ColorDescription\x1a\xe7\x0c\n\x0e\x43lassification\x12\x38\n\x04type\x18\x01 \x01(\x0e\x32*.osi3.StationaryObject.Classification.Type\x12@\n\x08material\x18\x02 \x01(\x0e\x32..osi3.StationaryObject.Classification.Material\x12>\n\x07\x64\x65nsity\x18\x03 \x01(\x0e\x32-.osi3.StationaryObject.Classification.Density\x12:\n\x05\x63olor\x18\x04 \x01(\x0e\x32+.osi3.StationaryObject.Classification.Color\x12\x66\n\x1c\x65mitting_structure_attribute\x18\x05 \x01(\x0b\x32@.osi3.StationaryObject.Classification.EmittingStructureAttribute\x12*\n\x10\x61ssigned_lane_id\x18\x06 \x03(\x0b\x32\x10.osi3.Identifier\x12 \n\x18\x61ssigned_lane_percentage\x18\x07 \x03(\x01\x12<\n\x17logical_lane_assignment\x18\x08 \x03(\x0b\x32\x1b.osi3.LogicalLaneAssignment\x1a\x91\x01\n\x1a\x45mittingStructureAttribute\x12-\n\x0fwavelength_data\x18\x01 \x03(\x0b\x32\x14.osi3.WavelengthData\x12\x44\n\x1f\x65mitted_spatial_signal_strength\x18\x03 \x03(\x0b\x32\x1b.osi3.SpatialSignalStrength\"\xa8\x03\n\x04Type\x12\x10\n\x0cTYPE_UNKNOWN\x10\x00\x12\x0e\n\nTYPE_OTHER\x10\x01\x12\x0f\n\x0bTYPE_BRIDGE\x10\x02\x12\x11\n\rTYPE_BUILDING\x10\x03\x12\r\n\tTYPE_POLE\x10\x04\x12\x0e\n\nTYPE_PYLON\x10\x05\x12\x13\n\x0fTYPE_DELINEATOR\x10\x06\x12\r\n\tTYPE_TREE\x10\x07\x12\x10\n\x0cTYPE_BARRIER\x10\x08\x12\x13\n\x0fTYPE_VEGETATION\x10\t\x12\x12\n\x0eTYPE_CURBSTONE\x10\n\x12\r\n\tTYPE_WALL\x10\x0b\x12\x1b\n\x17TYPE_VERTICAL_STRUCTURE\x10\x0c\x12\x1e\n\x1aTYPE_RECTANGULAR_STRUCTURE\x10\r\x12\x1b\n\x17TYPE_OVERHEAD_STRUCTURE\x10\x0e\x12\x1d\n\x19TYPE_REFLECTIVE_STRUCTURE\x10\x0f\x12\"\n\x1eTYPE_CONSTRUCTION_SITE_ELEMENT\x10\x10\x12\x13\n\x0fTYPE_SPEED_BUMP\x10\x11\x12\x1b\n\x17TYPE_EMITTING_STRUCTURE\x10\x12\"\xc1\x01\n\x08Material\x12\x14\n\x10MATERIAL_UNKNOWN\x10\x00\x12\x12\n\x0eMATERIAL_OTHER\x10\x01\x12\x11\n\rMATERIAL_WOOD\x10\x02\x12\x14\n\x10MATERIAL_PLASTIC\x10\x03\x12\x15\n\x11MATERIAL_CONCRETE\x10\x04\x12\x12\n\x0eMATERIAL_METAL\x10\x05\x12\x12\n\x0eMATERIAL_STONE\x10\x06\x12\x11\n\rMATERIAL_GLAS\x10\x07\x12\x10\n\x0cMATERIAL_MUD\x10\x08\"\x9f\x01\n\x07\x44\x65nsity\x12\x13\n\x0f\x44\x45NSITY_UNKNOWN\x10\x00\x12\x11\n\rDENSITY_OTHER\x10\x01\x12\x11\n\rDENSITY_SOLID\x10\x02\x12\x16\n\x12\x44\x45NSITY_SMALL_MESH\x10\x03\x12\x17\n\x13\x44\x45NSITY_MEDIAN_MESH\x10\x04\x12\x16\n\x12\x44\x45NSITY_LARGE_MESH\x10\x05\x12\x10\n\x0c\x44\x45NSITY_OPEN\x10\x06\"\xc3\x01\n\x05\x43olor\x12\x11\n\rCOLOR_UNKNOWN\x10\x00\x12\x0f\n\x0b\x43OLOR_OTHER\x10\x01\x12\x10\n\x0c\x43OLOR_YELLOW\x10\x02\x12\x0f\n\x0b\x43OLOR_GREEN\x10\x03\x12\x0e\n\nCOLOR_BLUE\x10\x04\x12\x10\n\x0c\x43OLOR_VIOLET\x10\x05\x12\r\n\tCOLOR_RED\x10\x06\x12\x10\n\x0c\x43OLOR_ORANGE\x10\x07\x12\x0f\n\x0b\x43OLOR_BLACK\x10\x08\x12\x0e\n\nCOLOR_GREY\x10\t\x12\x0f\n\x0b\x43OLOR_WHITE\x10\n\"\xa1\x1e\n\x0cMovingObject\x12\x1c\n\x02id\x18\x01 \x01(\x0b\x32\x10.osi3.Identifier\x12\x1e\n\x04\x62\x61se\x18\x02 \x01(\x0b\x32\x10.osi3.BaseMoving\x12%\n\x04type\x18\x03 \x01(\x0e\x32\x17.osi3.MovingObject.Type\x12*\n\x10\x61ssigned_lane_id\x18\x04 \x03(\x0b\x32\x10.osi3.Identifier\x12@\n\x12vehicle_attributes\x18\x05 \x01(\x0b\x32$.osi3.MovingObject.VehicleAttributes\x12H\n\x16vehicle_classification\x18\x06 \x01(\x0b\x32(.osi3.MovingObject.VehicleClassification\x12\x17\n\x0fmodel_reference\x18\x07 \x01(\t\x12+\n\x11\x66uture_trajectory\x18\x08 \x03(\x0b\x32\x10.osi3.StatePoint\x12S\n\x1cmoving_object_classification\x18\t \x01(\x0b\x32-.osi3.MovingObject.MovingObjectClassification\x12\x31\n\x10source_reference\x18\n \x03(\x0b\x32\x17.osi3.ExternalReference\x12\x31\n\x11\x63olor_description\x18\x0b \x01(\x0b\x32\x16.osi3.ColorDescription\x1a\xb4\x04\n\x11VehicleAttributes\x12#\n\tdriver_id\x18\x01 \x01(\x0b\x32\x10.osi3.Identifier\x12\x14\n\x0cradius_wheel\x18\x02 \x01(\x01\x12\x15\n\rnumber_wheels\x18\x03 \x01(\r\x12(\n\x10\x62\x62\x63\x65nter_to_rear\x18\x04 \x01(\x0b\x32\x0e.osi3.Vector3d\x12)\n\x11\x62\x62\x63\x65nter_to_front\x18\x05 \x01(\x0b\x32\x0e.osi3.Vector3d\x12\x18\n\x10ground_clearance\x18\x06 \x01(\x01\x12\x42\n\nwheel_data\x18\x07 \x03(\x0b\x32..osi3.MovingObject.VehicleAttributes.WheelData\x12\x1c\n\x14steering_wheel_angle\x18\x08 \x01(\x01\x1a\xfb\x01\n\tWheelData\x12\x0c\n\x04\x61xle\x18\x01 \x01(\r\x12\r\n\x05index\x18\x02 \x01(\r\x12 \n\x08position\x18\x03 \x01(\x0b\x32\x0e.osi3.Vector3d\x12\x14\n\x0cwheel_radius\x18\x04 \x01(\x01\x12\x12\n\nrim_radius\x18\x05 \x01(\x01\x12\r\n\x05width\x18\x06 \x01(\x01\x12(\n\x0borientation\x18\x07 \x01(\x0b\x32\x13.osi3.Orientation3d\x12\x15\n\rrotation_rate\x18\x08 \x01(\x01\x12\x17\n\x0fmodel_reference\x18\t \x01(\t\x12\x1c\n\x14\x66riction_coefficient\x18\n \x01(\x01\x1a\xa8\x01\n\x1aMovingObjectClassification\x12*\n\x10\x61ssigned_lane_id\x18\x01 \x03(\x0b\x32\x10.osi3.Identifier\x12 \n\x18\x61ssigned_lane_percentage\x18\x02 \x03(\x01\x12<\n\x17logical_lane_assignment\x18\x03 \x03(\x0b\x32\x1b.osi3.LogicalLaneAssignment\x1a\xae\x13\n\x15VehicleClassification\x12;\n\x04type\x18\x01 \x01(\x0e\x32-.osi3.MovingObject.VehicleClassification.Type\x12H\n\x0blight_state\x18\x02 \x01(\x0b\x32\x33.osi3.MovingObject.VehicleClassification.LightState\x12\x13\n\x0bhas_trailer\x18\x03 \x01(\x08\x12$\n\ntrailer_id\x18\x04 \x01(\x0b\x32\x10.osi3.Identifier\x12;\n\x04role\x18\x05 \x01(\x0e\x32-.osi3.MovingObject.VehicleClassification.Role\x1a\xd3\x0c\n\nLightState\x12[\n\x0findicator_state\x18\x01 \x01(\x0e\x32\x42.osi3.MovingObject.VehicleClassification.LightState.IndicatorState\x12^\n\x0f\x66ront_fog_light\x18\x02 \x01(\x0e\x32\x45.osi3.MovingObject.VehicleClassification.LightState.GenericLightState\x12]\n\x0erear_fog_light\x18\x03 \x01(\x0e\x32\x45.osi3.MovingObject.VehicleClassification.LightState.GenericLightState\x12Y\n\nhead_light\x18\x04 \x01(\x0e\x32\x45.osi3.MovingObject.VehicleClassification.LightState.GenericLightState\x12X\n\thigh_beam\x18\x05 \x01(\x0e\x32\x45.osi3.MovingObject.VehicleClassification.LightState.GenericLightState\x12^\n\x0freversing_light\x18\x06 \x01(\x0e\x32\x45.osi3.MovingObject.VehicleClassification.LightState.GenericLightState\x12^\n\x11\x62rake_light_state\x18\x07 \x01(\x0e\x32\x43.osi3.MovingObject.VehicleClassification.LightState.BrakeLightState\x12n\n\x1flicense_plate_illumination_rear\x18\x08 \x01(\x0e\x32\x45.osi3.MovingObject.VehicleClassification.LightState.GenericLightState\x12m\n\x1e\x65mergency_vehicle_illumination\x18\t \x01(\x0e\x32\x45.osi3.MovingObject.VehicleClassification.LightState.GenericLightState\x12k\n\x1cservice_vehicle_illumination\x18\n \x01(\x0e\x32\x45.osi3.MovingObject.VehicleClassification.LightState.GenericLightState\"\xb3\x01\n\x0eIndicatorState\x12\x1b\n\x17INDICATOR_STATE_UNKNOWN\x10\x00\x12\x19\n\x15INDICATOR_STATE_OTHER\x10\x01\x12\x17\n\x13INDICATOR_STATE_OFF\x10\x02\x12\x18\n\x14INDICATOR_STATE_LEFT\x10\x03\x12\x19\n\x15INDICATOR_STATE_RIGHT\x10\x04\x12\x1b\n\x17INDICATOR_STATE_WARNING\x10\x05\"\x8a\x02\n\x11GenericLightState\x12\x1f\n\x1bGENERIC_LIGHT_STATE_UNKNOWN\x10\x00\x12\x1d\n\x19GENERIC_LIGHT_STATE_OTHER\x10\x01\x12\x1b\n\x17GENERIC_LIGHT_STATE_OFF\x10\x02\x12\x1a\n\x16GENERIC_LIGHT_STATE_ON\x10\x03\x12%\n!GENERIC_LIGHT_STATE_FLASHING_BLUE\x10\x04\x12-\n)GENERIC_LIGHT_STATE_FLASHING_BLUE_AND_RED\x10\x05\x12&\n\"GENERIC_LIGHT_STATE_FLASHING_AMBER\x10\x06\"\xa4\x01\n\x0f\x42rakeLightState\x12\x1d\n\x19\x42RAKE_LIGHT_STATE_UNKNOWN\x10\x00\x12\x1b\n\x17\x42RAKE_LIGHT_STATE_OTHER\x10\x01\x12\x19\n\x15\x42RAKE_LIGHT_STATE_OFF\x10\x02\x12\x1c\n\x18\x42RAKE_LIGHT_STATE_NORMAL\x10\x03\x12\x1c\n\x18\x42RAKE_LIGHT_STATE_STRONG\x10\x04\"\xcf\x02\n\x04Type\x12\x10\n\x0cTYPE_UNKNOWN\x10\x00\x12\x0e\n\nTYPE_OTHER\x10\x01\x12\x12\n\x0eTYPE_SMALL_CAR\x10\x02\x12\x14\n\x10TYPE_COMPACT_CAR\x10\x03\x12\x13\n\x0fTYPE_MEDIUM_CAR\x10\x04\x12\x13\n\x0fTYPE_LUXURY_CAR\x10\x05\x12\x15\n\x11TYPE_DELIVERY_VAN\x10\x06\x12\x14\n\x10TYPE_HEAVY_TRUCK\x10\x07\x12\x14\n\x10TYPE_SEMITRACTOR\x10\x10\x12\x14\n\x10TYPE_SEMITRAILER\x10\x08\x12\x10\n\x0cTYPE_TRAILER\x10\t\x12\x12\n\x0eTYPE_MOTORBIKE\x10\n\x12\x10\n\x0cTYPE_BICYCLE\x10\x0b\x12\x0c\n\x08TYPE_BUS\x10\x0c\x12\r\n\tTYPE_TRAM\x10\r\x12\x0e\n\nTYPE_TRAIN\x10\x0e\x12\x13\n\x0fTYPE_WHEELCHAIR\x10\x0f\"\xed\x01\n\x04Role\x12\x10\n\x0cROLE_UNKNOWN\x10\x00\x12\x0e\n\nROLE_OTHER\x10\x01\x12\x0e\n\nROLE_CIVIL\x10\x02\x12\x12\n\x0eROLE_AMBULANCE\x10\x03\x12\r\n\tROLE_FIRE\x10\x04\x12\x0f\n\x0bROLE_POLICE\x10\x05\x12\x19\n\x15ROLE_PUBLIC_TRANSPORT\x10\x06\x12\x18\n\x14ROLE_ROAD_ASSISTANCE\x10\x07\x12\x1b\n\x17ROLE_GARBAGE_COLLECTION\x10\x08\x12\x1a\n\x16ROLE_ROAD_CONSTRUCTION\x10\t\x12\x11\n\rROLE_MILITARY\x10\n\"`\n\x04Type\x12\x10\n\x0cTYPE_UNKNOWN\x10\x00\x12\x0e\n\nTYPE_OTHER\x10\x01\x12\x10\n\x0cTYPE_VEHICLE\x10\x02\x12\x13\n\x0fTYPE_PEDESTRIAN\x10\x03\x12\x0f\n\x0bTYPE_ANIMAL\x10\x04\x42\x02H\x01')

_builder.BuildMessageAndEnumDescriptors(DESCRIPTOR, globals())
_builder.BuildTopDescriptorsAndMessages(DESCRIPTOR, 'osi3.osi_object_pb2', globals())
if _descriptor._USE_C_DESCRIPTORS == False:

  DESCRIPTOR._options = None
  DESCRIPTOR._serialized_options = b'H\001'
  _STATIONARYOBJECT._serialized_start=55
  _STATIONARYOBJECT._serialized_end=1971
  _STATIONARYOBJECT_CLASSIFICATION._serialized_start=332
  _STATIONARYOBJECT_CLASSIFICATION._serialized_end=1971
  _STATIONARYOBJECT_CLASSIFICATION_EMITTINGSTRUCTUREATTRIBUTE._serialized_start=843
  _STATIONARYOBJECT_CLASSIFICATION_EMITTINGSTRUCTUREATTRIBUTE._serialized_end=988
  _STATIONARYOBJECT_CLASSIFICATION_TYPE._serialized_start=991
  _STATIONARYOBJECT_CLASSIFICATION_TYPE._serialized_end=1415
  _STATIONARYOBJECT_CLASSIFICATION_MATERIAL._serialized_start=1418
  _STATIONARYOBJECT_CLASSIFICATION_MATERIAL._serialized_end=1611
  _STATIONARYOBJECT_CLASSIFICATION_DENSITY._serialized_start=1614
  _STATIONARYOBJECT_CLASSIFICATION_DENSITY._serialized_end=1773
  _STATIONARYOBJECT_CLASSIFICATION_COLOR._serialized_start=1776
  _STATIONARYOBJECT_CLASSIFICATION_COLOR._serialized_end=1971
  _MOVINGOBJECT._serialized_start=1974
  _MOVINGOBJECT._serialized_end=5847
  _MOVINGOBJECT_VEHICLEATTRIBUTES._serialized_start=2533
  _MOVINGOBJECT_VEHICLEATTRIBUTES._serialized_end=3097
  _MOVINGOBJECT_VEHICLEATTRIBUTES_WHEELDATA._serialized_start=2846
  _MOVINGOBJECT_VEHICLEATTRIBUTES_WHEELDATA._serialized_end=3097
  _MOVINGOBJECT_MOVINGOBJECTCLASSIFICATION._serialized_start=3100
  _MOVINGOBJECT_MOVINGOBJECTCLASSIFICATION._serialized_end=3268
  _MOVINGOBJECT_VEHICLECLASSIFICATION._serialized_start=3271
  _MOVINGOBJECT_VEHICLECLASSIFICATION._serialized_end=5749
  _MOVINGOBJECT_VEHICLECLASSIFICATION_LIGHTSTATE._serialized_start=3552
  _MOVINGOBJECT_VEHICLECLASSIFICATION_LIGHTSTATE._serialized_end=5171
  _MOVINGOBJECT_VEHICLECLASSIFICATION_LIGHTSTATE_INDICATORSTATE._serialized_start=4556
  _MOVINGOBJECT_VEHICLECLASSIFICATION_LIGHTSTATE_INDICATORSTATE._serialized_end=4735
  _MOVINGOBJECT_VEHICLECLASSIFICATION_LIGHTSTATE_GENERICLIGHTSTATE._serialized_start=4738
  _MOVINGOBJECT_VEHICLECLASSIFICATION_LIGHTSTATE_GENERICLIGHTSTATE._serialized_end=5004
  _MOVINGOBJECT_VEHICLECLASSIFICATION_LIGHTSTATE_BRAKELIGHTSTATE._serialized_start=5007
  _MOVINGOBJECT_VEHICLECLASSIFICATION_LIGHTSTATE_BRAKELIGHTSTATE._serialized_end=5171
  _MOVINGOBJECT_VEHICLECLASSIFICATION_TYPE._serialized_start=5174
  _MOVINGOBJECT_VEHICLECLASSIFICATION_TYPE._serialized_end=5509
  _MOVINGOBJECT_VEHICLECLASSIFICATION_ROLE._serialized_start=5512
  _MOVINGOBJECT_VEHICLECLASSIFICATION_ROLE._serialized_end=5749
  _MOVINGOBJECT_TYPE._serialized_start=5751
  _MOVINGOBJECT_TYPE._serialized_end=5847
# @@protoc_insertion_point(module_scope)
