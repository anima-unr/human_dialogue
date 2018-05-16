#include "pick_and_place/pr2_pick_and_place_service.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "pick_and_place_service");
  bool save = false, read = false, save_place = false;
  std::string save_file, read_file;
  char c;
  ros::NodeHandle nh;
  while ((c = getopt(argc, argv, "s:r:p:o:")) != -1) {
    switch (c) {
      case 's':
        save_file = optarg;
        save = true;
        break;
      case 'r':
        read_file = optarg;
        read = true;
        break;
      case 'p': // pick place saving 
        save_file = optarg;
        save_place = true;
        break;
      case 'o':
        // TODO: Do I need to set anything here?!
        break;
      case '?':
        if (isprint(optopt))
          printf("Unknown option: %d.\n", optopt);
        return 1;
      default:
        printf("Assuming the yolo-based vision manip pipeline is running. Will read from input calibration file for place locations.\n");
        read_file = optarg;
        // abort();
    }
  }

  // Create Pick Place object
  pr2::PickPlace pp("right_arm");

  // make choices
  if (read) {
    printf("Read File: %s\n", read_file.c_str());
    pp.ReadCalibration(read_file);
  } 
  else if ( save ) { 
    pp.CalibrateObjects();
  }
  else if (save_place) {
    pp.OnlineDetectionsPlaces();    
  }
  else{
    printf("Read File: %s\n", read_file.c_str());
    pp.ReadPlaces(read_file);      

    ros::AsyncSpinner spinner(1);
    spinner.start();

    pp.OnlineDetectionsPicks( nh );
  }

  // set params on the ros param server
  pp.PostParameters();

  // save the file if necessary
  if (save) {
    printf("Save File: %s\n", save_file.c_str());
    pp.SaveCalibration(save_file);
  }
  else if (save_place) {
    printf("Save File: %s\n", save_file.c_str());
    pp.SavePlaces(save_file);
  }


  // Advertise the service
  ros::ServiceServer service_object = nh.advertiseService(
    "pick_and_place_object",
    &pr2::PickPlace::PickAndPlaceObject,
    &pp);
  ros::ServiceServer service_check = nh.advertiseService(
    "pick_and_place_check",
    &pr2::PickPlace::PickAndPlacecheck,
    &pp);
  ros::ServiceServer service_state = nh.advertiseService(
    "pick_and_place_state",
    &pr2::PickPlace::PickAndPlaceState,
    &pp);
  ros::ServiceServer service_stop = nh.advertiseService(
    "pick_and_place_stop",
    &pr2::PickPlace::PickAndPlaceStop,
    &pp);
  printf("READY to PICK and Place\n");
  ros::spin();
  // while(ros::ok()) {
  //   ros::spinOnce();
  // }

  return 0;
}
