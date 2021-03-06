var searchIndex = JSON.parse('{\
"hamilton":{"doc":"","i":[[0,"driver","hamilton","",null,null],[0,"hamilton_lss_driver","hamilton::driver","",null,null],[3,"HamiltonLssDriver","hamilton::driver::hamilton_lss_driver","",null,null],[11,"new","","",0,[[["mutex",3],["arc",3],["bodyconfig",3]]]],[11,"send","","",0,[[["holonomicwheelcommand",3]]]],[11,"read_voltage","","",0,[[]]],[11,"set_color","","",0,[[["ledcolor",4]]]],[3,"MotorConfig","","",null,null],[3,"BodyConfig","","",null,null],[12,"left_front_controller","","",1,null],[12,"right_front_controller","","",1,null],[12,"left_rear_controller","","",1,null],[12,"right_rear_controller","","",1,null],[12,"multiplier","","",1,null],[11,"load","","",1,[[],["result",6]]],[11,"save","","",1,[[],["result",6]]],[11,"load_from_default","","Try to load config from default system location",1,[[],["result",6]]],[11,"save_to_default","","Save to default location",1,[[],["result",6]]],[11,"get_ids","","",1,[[]]],[11,"apply_commands_by_mapping","","",1,[[["holonomicwheelcommand",3]],["wiremovecommand",3]]],[3,"WireMoveCommand","","",null,null],[11,"new","","",2,[[]]],[11,"motors","","",2,[[]]],[3,"MotorCommand","hamilton::driver","",null,null],[11,"new","","",3,[[]]],[11,"id","","",3,[[]]],[11,"speed","","",3,[[]]],[0,"holonomic_controller","hamilton","",null,null],[3,"HolonomicWheelCommand","hamilton::holonomic_controller","",null,null],[11,"new","","",4,[[],["holonomicwheelcommand",3]]],[11,"from_move","","",4,[[],["holonomicwheelcommand",3]]],[11,"left_front","","",4,[[]]],[11,"right_front","","",4,[[]]],[11,"left_rear","","",4,[[]]],[11,"right_rear","","",4,[[]]],[11,"from","hamilton::driver::hamilton_lss_driver","",0,[[]]],[11,"into","","",0,[[]]],[11,"borrow","","",0,[[]]],[11,"borrow_mut","","",0,[[]]],[11,"try_from","","",0,[[],["result",4]]],[11,"try_into","","",0,[[],["result",4]]],[11,"type_id","","",0,[[],["typeid",3]]],[11,"to_subset","","",0,[[],["option",4]]],[11,"is_in_subset","","",0,[[]]],[11,"to_subset_unchecked","","",0,[[]]],[11,"from_subset","","",0,[[]]],[11,"vzip","","",0,[[]]],[11,"into_request","","",0,[[],["request",3]]],[11,"from","","",5,[[]]],[11,"into","","",5,[[]]],[11,"borrow","","",5,[[]]],[11,"borrow_mut","","",5,[[]]],[11,"try_from","","",5,[[],["result",4]]],[11,"try_into","","",5,[[],["result",4]]],[11,"type_id","","",5,[[],["typeid",3]]],[11,"to_subset","","",5,[[],["option",4]]],[11,"is_in_subset","","",5,[[]]],[11,"to_subset_unchecked","","",5,[[]]],[11,"from_subset","","",5,[[]]],[11,"vzip","","",5,[[]]],[11,"into_request","","",5,[[],["request",3]]],[11,"from","","",1,[[]]],[11,"into","","",1,[[]]],[11,"borrow","","",1,[[]]],[11,"borrow_mut","","",1,[[]]],[11,"try_from","","",1,[[],["result",4]]],[11,"try_into","","",1,[[],["result",4]]],[11,"type_id","","",1,[[],["typeid",3]]],[11,"to_subset","","",1,[[],["option",4]]],[11,"is_in_subset","","",1,[[]]],[11,"to_subset_unchecked","","",1,[[]]],[11,"from_subset","","",1,[[]]],[11,"vzip","","",1,[[]]],[11,"into_request","","",1,[[],["request",3]]],[11,"from","","",2,[[]]],[11,"into","","",2,[[]]],[11,"borrow","","",2,[[]]],[11,"borrow_mut","","",2,[[]]],[11,"try_from","","",2,[[],["result",4]]],[11,"try_into","","",2,[[],["result",4]]],[11,"type_id","","",2,[[],["typeid",3]]],[11,"to_subset","","",2,[[],["option",4]]],[11,"is_in_subset","","",2,[[]]],[11,"to_subset_unchecked","","",2,[[]]],[11,"from_subset","","",2,[[]]],[11,"vzip","","",2,[[]]],[11,"into_request","","",2,[[],["request",3]]],[11,"from","hamilton::driver","",3,[[]]],[11,"into","","",3,[[]]],[11,"borrow","","",3,[[]]],[11,"borrow_mut","","",3,[[]]],[11,"try_from","","",3,[[],["result",4]]],[11,"try_into","","",3,[[],["result",4]]],[11,"type_id","","",3,[[],["typeid",3]]],[11,"to_subset","","",3,[[],["option",4]]],[11,"is_in_subset","","",3,[[]]],[11,"to_subset_unchecked","","",3,[[]]],[11,"from_subset","","",3,[[]]],[11,"vzip","","",3,[[]]],[11,"into_request","","",3,[[],["request",3]]],[11,"from","hamilton::holonomic_controller","",4,[[]]],[11,"into","","",4,[[]]],[11,"borrow","","",4,[[]]],[11,"borrow_mut","","",4,[[]]],[11,"try_from","","",4,[[],["result",4]]],[11,"try_into","","",4,[[],["result",4]]],[11,"type_id","","",4,[[],["typeid",3]]],[11,"to_subset","","",4,[[],["option",4]]],[11,"is_in_subset","","",4,[[]]],[11,"to_subset_unchecked","","",4,[[]]],[11,"from_subset","","",4,[[]]],[11,"vzip","","",4,[[]]],[11,"into_request","","",4,[[],["request",3]]],[11,"default","hamilton::driver::hamilton_lss_driver","",5,[[],["motorconfig",3]]],[11,"default","","",1,[[],["bodyconfig",3]]],[11,"fmt","","",5,[[["formatter",3]],["result",6]]],[11,"fmt","","",1,[[["formatter",3]],["result",6]]],[11,"fmt","","",2,[[["formatter",3]],["result",6]]],[11,"fmt","hamilton::driver","",3,[[["formatter",3]],["result",6]]],[11,"serialize","hamilton::driver::hamilton_lss_driver","",5,[[],["result",4]]],[11,"serialize","","",1,[[],["result",4]]],[11,"deserialize","","",5,[[],["result",4]]],[11,"deserialize","","",1,[[],["result",4]]]],"p":[[3,"HamiltonLssDriver"],[3,"BodyConfig"],[3,"WireMoveCommand"],[3,"MotorCommand"],[3,"HolonomicWheelCommand"],[3,"MotorConfig"]]},\
"hamilton_controller":{"doc":"","i":[[0,"hamilton_service","hamilton_controller","",null,null],[0,"hamilton_remote_client","hamilton_controller::hamilton_service","Generated client implementations.",null,null],[3,"HamiltonRemoteClient","hamilton_controller::hamilton_service::hamilton_remote_client","",null,null],[12,"inner","","",0,null],[11,"connect","","Attempt to create a new client by connecting to a given …",0,[[]]],[11,"new","","",0,[[]]],[11,"with_interceptor","","",0,[[]]],[11,"move_stream","","",0,[[]]],[0,"hamilton_remote_server","hamilton_controller::hamilton_service","Generated server implementations.",null,null],[8,"HamiltonRemote","hamilton_controller::hamilton_service::hamilton_remote_server","Generated trait containing gRPC methods that should be …",null,null],[10,"move_stream","","",1,[[["streaming",3],["request",3]],[["pin",3],["box",3]]]],[3,"HamiltonRemoteServer","","",null,null],[12,"inner","","",2,null],[3,"_Inner","","",null,null],[12,"0","","",3,null],[12,"1","","",3,null],[11,"new","","",2,[[]]],[11,"with_interceptor","","",2,[[]]],[3,"MoveCommand","hamilton_controller::hamilton_service","",null,null],[12,"x","","",4,null],[12,"y","","",4,null],[12,"yaw","","",4,null],[3,"MoveRequest","","",null,null],[12,"command","","",5,null],[3,"MoveResponse","","",null,null],[3,"HamiltonRemoteController","hamilton_controller","",null,null],[12,"driver","","",6,null],[11,"new","","",6,[[["arc",3],["mutex",3]]]],[3,"Args","","",null,null],[12,"port","","",7,null],[12,"config","","",7,null],[5,"main","","",null,[[],["result",6]]],[11,"from","hamilton_controller::hamilton_service::hamilton_remote_client","",0,[[]]],[11,"into","","",0,[[]]],[11,"to_owned","","",0,[[]]],[11,"clone_into","","",0,[[]]],[11,"borrow","","",0,[[]]],[11,"borrow_mut","","",0,[[]]],[11,"try_from","","",0,[[],["result",4]]],[11,"try_into","","",0,[[],["result",4]]],[11,"type_id","","",0,[[],["typeid",3]]],[11,"into_request","","",0,[[],["request",3]]],[11,"vzip","","",0,[[]]],[11,"to_subset","","",0,[[],["option",4]]],[11,"is_in_subset","","",0,[[]]],[11,"to_subset_unchecked","","",0,[[]]],[11,"from_subset","","",0,[[]]],[11,"from","hamilton_controller::hamilton_service::hamilton_remote_server","",2,[[]]],[11,"into","","",2,[[]]],[11,"to_owned","","",2,[[]]],[11,"clone_into","","",2,[[]]],[11,"borrow","","",2,[[]]],[11,"borrow_mut","","",2,[[]]],[11,"try_from","","",2,[[],["result",4]]],[11,"try_into","","",2,[[],["result",4]]],[11,"type_id","","",2,[[],["typeid",3]]],[11,"poll_ready","","",2,[[["context",3]],[["poll",4],["result",4]]]],[11,"call","","",2,[[["request",3]]]],[11,"into_request","","",2,[[],["request",3]]],[11,"vzip","","",2,[[]]],[11,"to_subset","","",2,[[],["option",4]]],[11,"is_in_subset","","",2,[[]]],[11,"to_subset_unchecked","","",2,[[]]],[11,"from_subset","","",2,[[]]],[11,"from","","",3,[[]]],[11,"into","","",3,[[]]],[11,"to_owned","","",3,[[]]],[11,"clone_into","","",3,[[]]],[11,"borrow","","",3,[[]]],[11,"borrow_mut","","",3,[[]]],[11,"try_from","","",3,[[],["result",4]]],[11,"try_into","","",3,[[],["result",4]]],[11,"type_id","","",3,[[],["typeid",3]]],[11,"into_request","","",3,[[],["request",3]]],[11,"vzip","","",3,[[]]],[11,"to_subset","","",3,[[],["option",4]]],[11,"is_in_subset","","",3,[[]]],[11,"to_subset_unchecked","","",3,[[]]],[11,"from_subset","","",3,[[]]],[11,"from","hamilton_controller::hamilton_service","",4,[[]]],[11,"into","","",4,[[]]],[11,"to_owned","","",4,[[]]],[11,"clone_into","","",4,[[]]],[11,"borrow","","",4,[[]]],[11,"borrow_mut","","",4,[[]]],[11,"try_from","","",4,[[],["result",4]]],[11,"try_into","","",4,[[],["result",4]]],[11,"type_id","","",4,[[],["typeid",3]]],[11,"into_request","","",4,[[],["request",3]]],[11,"vzip","","",4,[[]]],[11,"to_subset","","",4,[[],["option",4]]],[11,"is_in_subset","","",4,[[]]],[11,"to_subset_unchecked","","",4,[[]]],[11,"from_subset","","",4,[[]]],[11,"from","","",5,[[]]],[11,"into","","",5,[[]]],[11,"to_owned","","",5,[[]]],[11,"clone_into","","",5,[[]]],[11,"borrow","","",5,[[]]],[11,"borrow_mut","","",5,[[]]],[11,"try_from","","",5,[[],["result",4]]],[11,"try_into","","",5,[[],["result",4]]],[11,"type_id","","",5,[[],["typeid",3]]],[11,"into_request","","",5,[[],["request",3]]],[11,"vzip","","",5,[[]]],[11,"to_subset","","",5,[[],["option",4]]],[11,"is_in_subset","","",5,[[]]],[11,"to_subset_unchecked","","",5,[[]]],[11,"from_subset","","",5,[[]]],[11,"from","","",8,[[]]],[11,"into","","",8,[[]]],[11,"to_owned","","",8,[[]]],[11,"clone_into","","",8,[[]]],[11,"borrow","","",8,[[]]],[11,"borrow_mut","","",8,[[]]],[11,"try_from","","",8,[[],["result",4]]],[11,"try_into","","",8,[[],["result",4]]],[11,"type_id","","",8,[[],["typeid",3]]],[11,"into_request","","",8,[[],["request",3]]],[11,"vzip","","",8,[[]]],[11,"to_subset","","",8,[[],["option",4]]],[11,"is_in_subset","","",8,[[]]],[11,"to_subset_unchecked","","",8,[[]]],[11,"from_subset","","",8,[[]]],[11,"from","hamilton_controller","",6,[[]]],[11,"into","","",6,[[]]],[11,"borrow","","",6,[[]]],[11,"borrow_mut","","",6,[[]]],[11,"try_from","","",6,[[],["result",4]]],[11,"try_into","","",6,[[],["result",4]]],[11,"type_id","","",6,[[],["typeid",3]]],[11,"into_request","","",6,[[],["request",3]]],[11,"vzip","","",6,[[]]],[11,"to_subset","","",6,[[],["option",4]]],[11,"is_in_subset","","",6,[[]]],[11,"to_subset_unchecked","","",6,[[]]],[11,"from_subset","","",6,[[]]],[11,"from","","",7,[[]]],[11,"into","","",7,[[]]],[11,"borrow","","",7,[[]]],[11,"borrow_mut","","",7,[[]]],[11,"try_from","","",7,[[],["result",4]]],[11,"try_into","","",7,[[],["result",4]]],[11,"type_id","","",7,[[],["typeid",3]]],[11,"into_request","","",7,[[],["request",3]]],[11,"vzip","","",7,[[]]],[11,"to_subset","","",7,[[],["option",4]]],[11,"is_in_subset","","",7,[[]]],[11,"to_subset_unchecked","","",7,[[]]],[11,"from_subset","","",7,[[]]],[11,"move_stream","","",6,[[["request",3],["streaming",3]],[["box",3],["pin",3]]]],[11,"clone","hamilton_controller::hamilton_service","",4,[[],["movecommand",3]]],[11,"clone","","",5,[[],["moverequest",3]]],[11,"clone","","",8,[[],["moveresponse",3]]],[11,"clone","hamilton_controller::hamilton_service::hamilton_remote_client","",0,[[]]],[11,"clone","hamilton_controller::hamilton_service::hamilton_remote_server","",2,[[]]],[11,"clone","","",3,[[]]],[11,"default","hamilton_controller::hamilton_service","",4,[[]]],[11,"default","","",5,[[]]],[11,"default","","",8,[[]]],[11,"eq","","",4,[[["movecommand",3]]]],[11,"ne","","",4,[[["movecommand",3]]]],[11,"eq","","",5,[[["moverequest",3]]]],[11,"ne","","",5,[[["moverequest",3]]]],[11,"eq","","",8,[[["moveresponse",3]]]],[11,"fmt","","",4,[[["formatter",3]],["result",6]]],[11,"fmt","","",5,[[["formatter",3]],["result",6]]],[11,"fmt","","",8,[[["formatter",3]],["result",6]]],[11,"fmt","hamilton_controller::hamilton_service::hamilton_remote_client","",0,[[["formatter",3]],["result",6]]],[11,"fmt","hamilton_controller::hamilton_service::hamilton_remote_server","",2,[[["formatter",3]],["result",6]]],[11,"fmt","","",3,[[["formatter",3]],["result",6]]],[11,"into_app","hamilton_controller","",7,[[],["app",3]]],[11,"augment_clap","","",7,[[["app",3]],["app",3]]],[11,"from_arg_matches","","",7,[[["argmatches",3]]]],[11,"poll_ready","hamilton_controller::hamilton_service::hamilton_remote_server","",2,[[["context",3]],[["poll",4],["result",4]]]],[11,"call","","",2,[[["request",3]]]],[11,"encode_raw","hamilton_controller::hamilton_service","",4,[[]]],[11,"merge_field","","",4,[[["wiretype",4],["decodecontext",3]],[["result",4],["decodeerror",3]]]],[11,"encoded_len","","",4,[[]]],[11,"clear","","",4,[[]]],[11,"encode_raw","","",5,[[]]],[11,"merge_field","","",5,[[["wiretype",4],["decodecontext",3]],[["result",4],["decodeerror",3]]]],[11,"encoded_len","","",5,[[]]],[11,"clear","","",5,[[]]],[11,"encode_raw","","",8,[[]]],[11,"merge_field","","",8,[[["wiretype",4],["decodecontext",3]],[["result",4],["decodeerror",3]]]],[11,"encoded_len","","",8,[[]]],[11,"clear","","",8,[[]]]],"p":[[3,"HamiltonRemoteClient"],[8,"HamiltonRemote"],[3,"HamiltonRemoteServer"],[3,"_Inner"],[3,"MoveCommand"],[3,"MoveRequest"],[3,"HamiltonRemoteController"],[3,"Args"],[3,"MoveResponse"]]},\
"hamilton_guppy_controller":{"doc":"","i":[[0,"hamilton_service","hamilton_guppy_controller","",null,null],[0,"hamilton_remote_client","hamilton_guppy_controller::hamilton_service","Generated client implementations.",null,null],[3,"HamiltonRemoteClient","hamilton_guppy_controller::hamilton_service::hamilton_remote_client","",null,null],[12,"inner","","",0,null],[11,"connect","","Attempt to create a new client by connecting to a given …",0,[[]]],[11,"new","","",0,[[]]],[11,"with_interceptor","","",0,[[]]],[11,"move_stream","","",0,[[]]],[0,"hamilton_remote_server","hamilton_guppy_controller::hamilton_service","Generated server implementations.",null,null],[8,"HamiltonRemote","hamilton_guppy_controller::hamilton_service::hamilton_remote_server","Generated trait containing gRPC methods that should be …",null,null],[10,"move_stream","","",1,[[["request",3],["streaming",3]],[["box",3],["pin",3]]]],[3,"HamiltonRemoteServer","","",null,null],[12,"inner","","",2,null],[3,"_Inner","","",null,null],[12,"0","","",3,null],[12,"1","","",3,null],[11,"new","","",2,[[]]],[11,"with_interceptor","","",2,[[]]],[3,"MoveCommand","hamilton_guppy_controller::hamilton_service","",null,null],[12,"x","","",4,null],[12,"y","","",4,null],[12,"yaw","","",4,null],[3,"MoveRequest","","",null,null],[12,"command","","",5,null],[3,"MoveResponse","","",null,null],[3,"HamiltonRemoteController","hamilton_guppy_controller","",null,null],[12,"driver","","",6,null],[11,"new","","",6,[[["arc",3],["mutex",3]]]],[5,"connect_guppy","","",null,[[["arc",3],["mutex",3]]]],[3,"Args","","",null,null],[12,"port","","",7,null],[12,"config","","",7,null],[5,"main","","",null,[[],["result",6]]],[11,"from","hamilton_guppy_controller::hamilton_service::hamilton_remote_client","",0,[[]]],[11,"into","","",0,[[]]],[11,"to_owned","","",0,[[]]],[11,"clone_into","","",0,[[]]],[11,"borrow","","",0,[[]]],[11,"borrow_mut","","",0,[[]]],[11,"try_from","","",0,[[],["result",4]]],[11,"try_into","","",0,[[],["result",4]]],[11,"type_id","","",0,[[],["typeid",3]]],[11,"to_subset","","",0,[[],["option",4]]],[11,"is_in_subset","","",0,[[]]],[11,"to_subset_unchecked","","",0,[[]]],[11,"from_subset","","",0,[[]]],[11,"vzip","","",0,[[]]],[11,"into_request","","",0,[[],["request",3]]],[11,"from","hamilton_guppy_controller::hamilton_service::hamilton_remote_server","",2,[[]]],[11,"into","","",2,[[]]],[11,"to_owned","","",2,[[]]],[11,"clone_into","","",2,[[]]],[11,"borrow","","",2,[[]]],[11,"borrow_mut","","",2,[[]]],[11,"try_from","","",2,[[],["result",4]]],[11,"try_into","","",2,[[],["result",4]]],[11,"type_id","","",2,[[],["typeid",3]]],[11,"to_subset","","",2,[[],["option",4]]],[11,"is_in_subset","","",2,[[]]],[11,"to_subset_unchecked","","",2,[[]]],[11,"from_subset","","",2,[[]]],[11,"vzip","","",2,[[]]],[11,"poll_ready","","",2,[[["context",3]],[["poll",4],["result",4]]]],[11,"call","","",2,[[["request",3]]]],[11,"into_request","","",2,[[],["request",3]]],[11,"from","","",3,[[]]],[11,"into","","",3,[[]]],[11,"to_owned","","",3,[[]]],[11,"clone_into","","",3,[[]]],[11,"borrow","","",3,[[]]],[11,"borrow_mut","","",3,[[]]],[11,"try_from","","",3,[[],["result",4]]],[11,"try_into","","",3,[[],["result",4]]],[11,"type_id","","",3,[[],["typeid",3]]],[11,"to_subset","","",3,[[],["option",4]]],[11,"is_in_subset","","",3,[[]]],[11,"to_subset_unchecked","","",3,[[]]],[11,"from_subset","","",3,[[]]],[11,"vzip","","",3,[[]]],[11,"into_request","","",3,[[],["request",3]]],[11,"from","hamilton_guppy_controller::hamilton_service","",4,[[]]],[11,"into","","",4,[[]]],[11,"to_owned","","",4,[[]]],[11,"clone_into","","",4,[[]]],[11,"borrow","","",4,[[]]],[11,"borrow_mut","","",4,[[]]],[11,"try_from","","",4,[[],["result",4]]],[11,"try_into","","",4,[[],["result",4]]],[11,"type_id","","",4,[[],["typeid",3]]],[11,"to_subset","","",4,[[],["option",4]]],[11,"is_in_subset","","",4,[[]]],[11,"to_subset_unchecked","","",4,[[]]],[11,"from_subset","","",4,[[]]],[11,"vzip","","",4,[[]]],[11,"into_request","","",4,[[],["request",3]]],[11,"from","","",5,[[]]],[11,"into","","",5,[[]]],[11,"to_owned","","",5,[[]]],[11,"clone_into","","",5,[[]]],[11,"borrow","","",5,[[]]],[11,"borrow_mut","","",5,[[]]],[11,"try_from","","",5,[[],["result",4]]],[11,"try_into","","",5,[[],["result",4]]],[11,"type_id","","",5,[[],["typeid",3]]],[11,"to_subset","","",5,[[],["option",4]]],[11,"is_in_subset","","",5,[[]]],[11,"to_subset_unchecked","","",5,[[]]],[11,"from_subset","","",5,[[]]],[11,"vzip","","",5,[[]]],[11,"into_request","","",5,[[],["request",3]]],[11,"from","","",8,[[]]],[11,"into","","",8,[[]]],[11,"to_owned","","",8,[[]]],[11,"clone_into","","",8,[[]]],[11,"borrow","","",8,[[]]],[11,"borrow_mut","","",8,[[]]],[11,"try_from","","",8,[[],["result",4]]],[11,"try_into","","",8,[[],["result",4]]],[11,"type_id","","",8,[[],["typeid",3]]],[11,"to_subset","","",8,[[],["option",4]]],[11,"is_in_subset","","",8,[[]]],[11,"to_subset_unchecked","","",8,[[]]],[11,"from_subset","","",8,[[]]],[11,"vzip","","",8,[[]]],[11,"into_request","","",8,[[],["request",3]]],[11,"from","hamilton_guppy_controller","",6,[[]]],[11,"into","","",6,[[]]],[11,"borrow","","",6,[[]]],[11,"borrow_mut","","",6,[[]]],[11,"try_from","","",6,[[],["result",4]]],[11,"try_into","","",6,[[],["result",4]]],[11,"type_id","","",6,[[],["typeid",3]]],[11,"to_subset","","",6,[[],["option",4]]],[11,"is_in_subset","","",6,[[]]],[11,"to_subset_unchecked","","",6,[[]]],[11,"from_subset","","",6,[[]]],[11,"vzip","","",6,[[]]],[11,"into_request","","",6,[[],["request",3]]],[11,"from","","",7,[[]]],[11,"into","","",7,[[]]],[11,"borrow","","",7,[[]]],[11,"borrow_mut","","",7,[[]]],[11,"try_from","","",7,[[],["result",4]]],[11,"try_into","","",7,[[],["result",4]]],[11,"type_id","","",7,[[],["typeid",3]]],[11,"to_subset","","",7,[[],["option",4]]],[11,"is_in_subset","","",7,[[]]],[11,"to_subset_unchecked","","",7,[[]]],[11,"from_subset","","",7,[[]]],[11,"vzip","","",7,[[]]],[11,"into_request","","",7,[[],["request",3]]],[11,"move_stream","","",6,[[["streaming",3],["request",3]],[["pin",3],["box",3]]]],[11,"clone","hamilton_guppy_controller::hamilton_service","",4,[[],["movecommand",3]]],[11,"clone","","",5,[[],["moverequest",3]]],[11,"clone","","",8,[[],["moveresponse",3]]],[11,"clone","hamilton_guppy_controller::hamilton_service::hamilton_remote_client","",0,[[]]],[11,"clone","hamilton_guppy_controller::hamilton_service::hamilton_remote_server","",2,[[]]],[11,"clone","","",3,[[]]],[11,"default","hamilton_guppy_controller::hamilton_service","",4,[[]]],[11,"default","","",5,[[]]],[11,"default","","",8,[[]]],[11,"eq","","",4,[[["movecommand",3]]]],[11,"ne","","",4,[[["movecommand",3]]]],[11,"eq","","",5,[[["moverequest",3]]]],[11,"ne","","",5,[[["moverequest",3]]]],[11,"eq","","",8,[[["moveresponse",3]]]],[11,"fmt","","",4,[[["formatter",3]],["result",6]]],[11,"fmt","","",5,[[["formatter",3]],["result",6]]],[11,"fmt","","",8,[[["formatter",3]],["result",6]]],[11,"fmt","hamilton_guppy_controller::hamilton_service::hamilton_remote_client","",0,[[["formatter",3]],["result",6]]],[11,"fmt","hamilton_guppy_controller::hamilton_service::hamilton_remote_server","",2,[[["formatter",3]],["result",6]]],[11,"fmt","","",3,[[["formatter",3]],["result",6]]],[11,"into_app","hamilton_guppy_controller","",7,[[],["app",3]]],[11,"augment_clap","","",7,[[["app",3]],["app",3]]],[11,"from_arg_matches","","",7,[[["argmatches",3]]]],[11,"poll_ready","hamilton_guppy_controller::hamilton_service::hamilton_remote_server","",2,[[["context",3]],[["poll",4],["result",4]]]],[11,"call","","",2,[[["request",3]]]],[11,"encode_raw","hamilton_guppy_controller::hamilton_service","",4,[[]]],[11,"merge_field","","",4,[[["decodecontext",3],["wiretype",4]],[["result",4],["decodeerror",3]]]],[11,"encoded_len","","",4,[[]]],[11,"clear","","",4,[[]]],[11,"encode_raw","","",5,[[]]],[11,"merge_field","","",5,[[["decodecontext",3],["wiretype",4]],[["result",4],["decodeerror",3]]]],[11,"encoded_len","","",5,[[]]],[11,"clear","","",5,[[]]],[11,"encode_raw","","",8,[[]]],[11,"merge_field","","",8,[[["decodecontext",3],["wiretype",4]],[["result",4],["decodeerror",3]]]],[11,"encoded_len","","",8,[[]]],[11,"clear","","",8,[[]]]],"p":[[3,"HamiltonRemoteClient"],[8,"HamiltonRemote"],[3,"HamiltonRemoteServer"],[3,"_Inner"],[3,"MoveCommand"],[3,"MoveRequest"],[3,"HamiltonRemoteController"],[3,"Args"],[3,"MoveResponse"]]},\
"vehicle_config_test":{"doc":"","i":[[3,"Args","vehicle_config_test","",null,null],[12,"port","","",0,null],[12,"test","","",0,null],[12,"move_test","","",0,null],[12,"config","","",0,null],[5,"main","","",null,[[],["result",6]]],[5,"wheels_test","","",null,[[["hamiltonlssdriver",3]]]],[5,"move_test","","",null,[[["hamiltonlssdriver",3]]]],[3,"MoveCommand","","",null,null],[12,"x","","",1,null],[12,"y","","",1,null],[12,"yaw","","",1,null],[11,"from","","",0,[[]]],[11,"into","","",0,[[]]],[11,"borrow","","",0,[[]]],[11,"borrow_mut","","",0,[[]]],[11,"try_from","","",0,[[],["result",4]]],[11,"try_into","","",0,[[],["result",4]]],[11,"type_id","","",0,[[],["typeid",3]]],[11,"into_request","","",0,[[],["request",3]]],[11,"vzip","","",0,[[]]],[11,"to_subset","","",0,[[],["option",4]]],[11,"is_in_subset","","",0,[[]]],[11,"to_subset_unchecked","","",0,[[]]],[11,"from_subset","","",0,[[]]],[11,"from","","",1,[[]]],[11,"into","","",1,[[]]],[11,"borrow","","",1,[[]]],[11,"borrow_mut","","",1,[[]]],[11,"try_from","","",1,[[],["result",4]]],[11,"try_into","","",1,[[],["result",4]]],[11,"type_id","","",1,[[],["typeid",3]]],[11,"into_request","","",1,[[],["request",3]]],[11,"vzip","","",1,[[]]],[11,"to_subset","","",1,[[],["option",4]]],[11,"is_in_subset","","",1,[[]]],[11,"to_subset_unchecked","","",1,[[]]],[11,"from_subset","","",1,[[]]],[11,"into_app","","",0,[[],["app",3]]],[11,"augment_clap","","",0,[[["app",3]],["app",3]]],[11,"from_arg_matches","","",0,[[["argmatches",3]]]]],"p":[[3,"Args"],[3,"MoveCommand"]]}\
}');
addSearchOptions(searchIndex);initSearch(searchIndex);