var searchIndex = JSON.parse('{\
"hamilton":{"doc":"","i":[[3,"HamiltonRemoteController","hamilton","",null,null],[12,"driver","","",0,null],[12,"motor_mapping","","",0,null],[3,"Args","","",null,null],[12,"port","","",1,null],[12,"test","","",1,null],[12,"move_test","","",1,null],[12,"config","","",1,null],[5,"main","","",null,[[],["result",6]]],[5,"wheels_test","","",null,[[["box",3],["motormapping",3]]]],[5,"move_test","","",null,[[["box",3],["motormapping",3]]]],[0,"driver","","",null,null],[3,"WireMoveCommand","hamilton::driver","",null,null],[12,"wheel_a","","",2,null],[12,"wheel_b","","",2,null],[12,"wheel_c","","",2,null],[12,"wheel_d","","",2,null],[0,"hamilton_lss_driver","","",null,null],[3,"HamiltonLssDriver","hamilton::driver::hamilton_lss_driver","",null,null],[12,"driver","","",3,null],[11,"new","","",3,[[]]],[8,"HamiltonDriver","hamilton::driver","",null,null],[10,"send","","",4,[[["wiremovecommand",3]],[["box",3],["pin",3]]]],[10,"read_voltage","","",4,[[],[["box",3],["pin",3]]]],[10,"set_color","","",4,[[["ledcolor",4]],[["box",3],["pin",3]]]],[0,"holonomic_controller","hamilton","",null,null],[3,"HolonomicWheelCommand","hamilton::holonomic_controller","",null,null],[12,"left_front","","",5,null],[12,"right_front","","",5,null],[12,"left_rear","","",5,null],[12,"right_rear","","",5,null],[3,"MotorMapping","","",null,null],[12,"left_front_controller","","",6,null],[12,"right_front_controller","","",6,null],[12,"left_rear_controller","","",6,null],[12,"right_rear_controller","","",6,null],[12,"multiplier","","",6,null],[4,"MotorMappingFlags","","",null,null],[13,"A","","",7,null],[13,"B","","",7,null],[13,"C","","",7,null],[13,"D","","",7,null],[8,"Clampable","","",null,null],[10,"clamp_num","","",8,[[]]],[11,"new","","",5,[[],["holonomicwheelcommand",3]]],[11,"load","","",6,[[],["result",6]]],[11,"save","","",6,[[],["result",6]]],[11,"load_from_default","","Try to load config from default system location",6,[[],["result",6]]],[11,"save_to_default","","Save to default location",6,[[],["result",6]]],[11,"apply_commands_by_mapping","","",6,[[["holonomicwheelcommand",3]],["wiremovecommand",3]]],[0,"hamilton","hamilton","",null,null],[3,"MoveCommand","hamilton::hamilton","",null,null],[12,"x","","",9,null],[12,"y","","",9,null],[12,"yaw","","",9,null],[3,"MoveRequest","","",null,null],[12,"command","","",10,null],[3,"MoveResponse","","",null,null],[0,"hamilton_remote_client","","Generated client implementations.",null,null],[3,"HamiltonRemoteClient","hamilton::hamilton::hamilton_remote_client","",null,null],[12,"inner","","",11,null],[11,"connect","","Attempt to create a new client by connecting to a given…",11,[[]]],[11,"new","","",11,[[]]],[11,"with_interceptor","","",11,[[]]],[11,"move_stream","","",11,[[]]],[0,"hamilton_remote_server","hamilton::hamilton","Generated server implementations.",null,null],[3,"HamiltonRemoteServer","hamilton::hamilton::hamilton_remote_server","",null,null],[12,"inner","","",12,null],[3,"_Inner","","",null,null],[12,"0","","",13,null],[12,"1","","",13,null],[8,"HamiltonRemote","","Generated trait containing gRPC methods that should be…",null,null],[10,"move_stream","","",14,[[["request",3],["streaming",3]],[["box",3],["pin",3]]]],[11,"new","","",12,[[]]],[11,"with_interceptor","","",12,[[]]],[11,"new","hamilton","",0,[[["mutex",3],["arc",3],["motormapping",3]]]],[11,"from","","",0,[[]]],[11,"into","","",0,[[]]],[11,"borrow","","",0,[[]]],[11,"borrow_mut","","",0,[[]]],[11,"try_from","","",0,[[],["result",4]]],[11,"try_into","","",0,[[],["result",4]]],[11,"type_id","","",0,[[],["typeid",3]]],[11,"into_request","","",0,[[],["request",3]]],[11,"vzip","","",0,[[]]],[11,"from","","",1,[[]]],[11,"into","","",1,[[]]],[11,"borrow","","",1,[[]]],[11,"borrow_mut","","",1,[[]]],[11,"try_from","","",1,[[],["result",4]]],[11,"try_into","","",1,[[],["result",4]]],[11,"type_id","","",1,[[],["typeid",3]]],[11,"into_request","","",1,[[],["request",3]]],[11,"vzip","","",1,[[]]],[11,"from","hamilton::driver","",2,[[]]],[11,"into","","",2,[[]]],[11,"borrow","","",2,[[]]],[11,"borrow_mut","","",2,[[]]],[11,"try_from","","",2,[[],["result",4]]],[11,"try_into","","",2,[[],["result",4]]],[11,"type_id","","",2,[[],["typeid",3]]],[11,"into_request","","",2,[[],["request",3]]],[11,"vzip","","",2,[[]]],[11,"from","hamilton::driver::hamilton_lss_driver","",3,[[]]],[11,"into","","",3,[[]]],[11,"borrow","","",3,[[]]],[11,"borrow_mut","","",3,[[]]],[11,"try_from","","",3,[[],["result",4]]],[11,"try_into","","",3,[[],["result",4]]],[11,"type_id","","",3,[[],["typeid",3]]],[11,"into_request","","",3,[[],["request",3]]],[11,"vzip","","",3,[[]]],[11,"from","hamilton::holonomic_controller","",5,[[]]],[11,"into","","",5,[[]]],[11,"borrow","","",5,[[]]],[11,"borrow_mut","","",5,[[]]],[11,"try_from","","",5,[[],["result",4]]],[11,"try_into","","",5,[[],["result",4]]],[11,"type_id","","",5,[[],["typeid",3]]],[11,"into_request","","",5,[[],["request",3]]],[11,"vzip","","",5,[[]]],[11,"from","","",6,[[]]],[11,"into","","",6,[[]]],[11,"borrow","","",6,[[]]],[11,"borrow_mut","","",6,[[]]],[11,"try_from","","",6,[[],["result",4]]],[11,"try_into","","",6,[[],["result",4]]],[11,"type_id","","",6,[[],["typeid",3]]],[11,"into_request","","",6,[[],["request",3]]],[11,"vzip","","",6,[[]]],[11,"from","","",7,[[]]],[11,"into","","",7,[[]]],[11,"borrow","","",7,[[]]],[11,"borrow_mut","","",7,[[]]],[11,"try_from","","",7,[[],["result",4]]],[11,"try_into","","",7,[[],["result",4]]],[11,"type_id","","",7,[[],["typeid",3]]],[11,"into_request","","",7,[[],["request",3]]],[11,"vzip","","",7,[[]]],[11,"from","hamilton::hamilton","",9,[[]]],[11,"into","","",9,[[]]],[11,"to_owned","","",9,[[]]],[11,"clone_into","","",9,[[]]],[11,"borrow","","",9,[[]]],[11,"borrow_mut","","",9,[[]]],[11,"try_from","","",9,[[],["result",4]]],[11,"try_into","","",9,[[],["result",4]]],[11,"type_id","","",9,[[],["typeid",3]]],[11,"into_request","","",9,[[],["request",3]]],[11,"vzip","","",9,[[]]],[11,"from","","",10,[[]]],[11,"into","","",10,[[]]],[11,"to_owned","","",10,[[]]],[11,"clone_into","","",10,[[]]],[11,"borrow","","",10,[[]]],[11,"borrow_mut","","",10,[[]]],[11,"try_from","","",10,[[],["result",4]]],[11,"try_into","","",10,[[],["result",4]]],[11,"type_id","","",10,[[],["typeid",3]]],[11,"into_request","","",10,[[],["request",3]]],[11,"vzip","","",10,[[]]],[11,"from","","",15,[[]]],[11,"into","","",15,[[]]],[11,"to_owned","","",15,[[]]],[11,"clone_into","","",15,[[]]],[11,"borrow","","",15,[[]]],[11,"borrow_mut","","",15,[[]]],[11,"try_from","","",15,[[],["result",4]]],[11,"try_into","","",15,[[],["result",4]]],[11,"type_id","","",15,[[],["typeid",3]]],[11,"into_request","","",15,[[],["request",3]]],[11,"vzip","","",15,[[]]],[11,"from","hamilton::hamilton::hamilton_remote_client","",11,[[]]],[11,"into","","",11,[[]]],[11,"to_owned","","",11,[[]]],[11,"clone_into","","",11,[[]]],[11,"borrow","","",11,[[]]],[11,"borrow_mut","","",11,[[]]],[11,"try_from","","",11,[[],["result",4]]],[11,"try_into","","",11,[[],["result",4]]],[11,"type_id","","",11,[[],["typeid",3]]],[11,"into_request","","",11,[[],["request",3]]],[11,"vzip","","",11,[[]]],[11,"from","hamilton::hamilton::hamilton_remote_server","",12,[[]]],[11,"into","","",12,[[]]],[11,"to_owned","","",12,[[]]],[11,"clone_into","","",12,[[]]],[11,"borrow","","",12,[[]]],[11,"borrow_mut","","",12,[[]]],[11,"try_from","","",12,[[],["result",4]]],[11,"try_into","","",12,[[],["result",4]]],[11,"type_id","","",12,[[],["typeid",3]]],[11,"poll_ready","","",12,[[["context",3]],[["result",4],["poll",4]]]],[11,"call","","",12,[[["request",3]]]],[11,"into_request","","",12,[[],["request",3]]],[11,"vzip","","",12,[[]]],[11,"from","","",13,[[]]],[11,"into","","",13,[[]]],[11,"to_owned","","",13,[[]]],[11,"clone_into","","",13,[[]]],[11,"borrow","","",13,[[]]],[11,"borrow_mut","","",13,[[]]],[11,"try_from","","",13,[[],["result",4]]],[11,"try_into","","",13,[[],["result",4]]],[11,"type_id","","",13,[[],["typeid",3]]],[11,"into_request","","",13,[[],["request",3]]],[11,"vzip","","",13,[[]]],[11,"send","hamilton::driver::hamilton_lss_driver","",3,[[["wiremovecommand",3]],[["box",3],["pin",3]]]],[11,"read_voltage","","",3,[[],[["box",3],["pin",3]]]],[11,"set_color","","",3,[[["ledcolor",4]],[["box",3],["pin",3]]]],[11,"move_stream","hamilton","",0,[[["request",3],["streaming",3]],[["box",3],["pin",3]]]],[11,"from","hamilton::holonomic_controller","",5,[[["movecommand",3]]]],[11,"clone","hamilton::hamilton","",9,[[],["movecommand",3]]],[11,"clone","","",10,[[],["moverequest",3]]],[11,"clone","","",15,[[],["moveresponse",3]]],[11,"clone","hamilton::hamilton::hamilton_remote_client","",11,[[]]],[11,"clone","hamilton::hamilton::hamilton_remote_server","",12,[[]]],[11,"clone","","",13,[[]]],[11,"default","hamilton::driver","",2,[[],["wiremovecommand",3]]],[11,"default","hamilton::holonomic_controller","",6,[[]]],[11,"default","hamilton::hamilton","",9,[[]]],[11,"default","","",10,[[]]],[11,"default","","",15,[[]]],[11,"eq","","",9,[[["movecommand",3]]]],[11,"ne","","",9,[[["movecommand",3]]]],[11,"eq","","",10,[[["moverequest",3]]]],[11,"ne","","",10,[[["moverequest",3]]]],[11,"eq","","",15,[[["moveresponse",3]]]],[11,"fmt","hamilton::driver","",2,[[["formatter",3]],["result",6]]],[11,"fmt","hamilton::holonomic_controller","",7,[[["formatter",3]],["result",6]]],[11,"fmt","","",6,[[["formatter",3]],["result",6]]],[11,"fmt","hamilton::hamilton","",9,[[["formatter",3]],["result",6]]],[11,"fmt","","",10,[[["formatter",3]],["result",6]]],[11,"fmt","","",15,[[["formatter",3]],["result",6]]],[11,"fmt","hamilton::hamilton::hamilton_remote_client","",11,[[["formatter",3]],["result",6]]],[11,"fmt","hamilton::hamilton::hamilton_remote_server","",12,[[["formatter",3]],["result",6]]],[11,"fmt","","",13,[[["formatter",3]],["result",6]]],[11,"serialize","hamilton::holonomic_controller","",7,[[],["result",4]]],[11,"serialize","","",6,[[],["result",4]]],[11,"deserialize","","",7,[[],["result",4]]],[11,"deserialize","","",6,[[],["result",4]]],[11,"into_app","hamilton","",1,[[],["app",3]]],[11,"augment_clap","","",1,[[["app",3]],["app",3]]],[11,"from_arg_matches","","",1,[[["argmatches",3]]]],[11,"poll_ready","hamilton::hamilton::hamilton_remote_server","",12,[[["context",3]],[["result",4],["poll",4]]]],[11,"call","","",12,[[["request",3]]]],[11,"encode_raw","hamilton::hamilton","",9,[[]]],[11,"merge_field","","",9,[[["wiretype",4],["decodecontext",3]],[["result",4],["decodeerror",3]]]],[11,"encoded_len","","",9,[[]]],[11,"clear","","",9,[[]]],[11,"encode_raw","","",10,[[]]],[11,"merge_field","","",10,[[["wiretype",4],["decodecontext",3]],[["result",4],["decodeerror",3]]]],[11,"encoded_len","","",10,[[]]],[11,"clear","","",10,[[]]],[11,"encode_raw","","",15,[[]]],[11,"merge_field","","",15,[[["wiretype",4],["decodecontext",3]],[["result",4],["decodeerror",3]]]],[11,"encoded_len","","",15,[[]]],[11,"clear","","",15,[[]]]],"p":[[3,"HamiltonRemoteController"],[3,"Args"],[3,"WireMoveCommand"],[3,"HamiltonLssDriver"],[8,"HamiltonDriver"],[3,"HolonomicWheelCommand"],[3,"MotorMapping"],[4,"MotorMappingFlags"],[8,"Clampable"],[3,"MoveCommand"],[3,"MoveRequest"],[3,"HamiltonRemoteClient"],[3,"HamiltonRemoteServer"],[3,"_Inner"],[8,"HamiltonRemote"],[3,"MoveResponse"]]}\
}');
addSearchOptions(searchIndex);initSearch(searchIndex);