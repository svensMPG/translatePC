#include <iostream>

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>


#include "./file_io.hpp"

// This function displays the help


// This is the main function
int
main (int argc, char** argv)
{

  // Show help
  if (pcl::console::find_switch (argc, argv, "-h") || pcl::console::find_switch (argc, argv, "--help")) {
    showHelp (argv[0]);
    return 0;
  }

  // Fetch point cloud filename in arguments | Works with PCD and PLY files
  std::vector<int> filenames;
  bool file_is_pcd = false;

  filenames = pcl::console::parse_file_extension_argument (argc, argv, ".ply");

  if (filenames.size () != 1)  {
    filenames = pcl::console::parse_file_extension_argument (argc, argv, ".pcd");

    if (filenames.size () != 1) {
      showHelp (argv[0]);
      return -1;
    } else {
      file_is_pcd = true;
    }
  }


  double minX = 0, minY  = 0 ,minZ  = 0;

  bool minXSpecified = pcl::console::find_switch (argc, argv, "-x");
  if (minXSpecified){
      pcl::console::parse (argc, argv, "-x", minX);
  }

  bool minYSpecified = pcl::console::find_switch (argc, argv, "-y");
  if (minYSpecified){
      pcl::console::parse (argc, argv, "-y", minY);
  }

  bool minZSpecified = pcl::console::find_switch (argc, argv, "-z");
  if (minZSpecified){
      pcl::console::parse (argc, argv, "-z", minZ);
  }

  const std::vector<double> minXYZValues = {minX,minY,minZ};


  // Load file | Works with PCD and PLY files
  pcl::PointCloud<pcl::PointXYZI>::Ptr source_cloud (new pcl::PointCloud<pcl::PointXYZI> ());
   pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZI> ());

  if (file_is_pcd) {
    if (pcl::io::loadPCDFile (argv[filenames[0]], *source_cloud) < 0)  {
      std::cout << "Error loading point cloud " << argv[filenames[0]] << std::endl << std::endl;
      showHelp (argv[0]);
      return -1;
    }
  } else {
    if (pcl::io::loadPLYFile (argv[filenames[0]], *source_cloud) < 0)  {
      std::cout << "Error loading point cloud " << argv[filenames[0]] << std::endl << std::endl;
      showHelp (argv[0]);
      return -1;
    }
  }

  std::cout << "provided minvalues are:\n x =  " << minXYZValues[0] << "\n y = " << minXYZValues[1] << "\n z = " << minXYZValues[2] << std::endl;
    pcl::io::savePCDFileASCII("bunny01.pcd", *source_cloud);


   addMinValues<pcl::PointXYZI>(source_cloud,transformed_cloud,minXYZValues);
   pcl::io::savePCDFileASCII("Transformed.pcd", *transformed_cloud);

   std::vector < std::vector< double > > pointVec;
   std::vector < unsigned int > intensityVec;

   addMinValues<pcl::PointXYZI>(source_cloud,pointVec,intensityVec,minXYZValues);
   toTXT("transformedPoints", pointVec, intensityVec);


  std::cout << "end reached!";
  return 0;
}
