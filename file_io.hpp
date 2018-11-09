#include <iostream>

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>

void
showHelp(char * program_name)
{
  std::cout << std::endl;
  std::cout << "Usage: " << program_name << " cloud_filename.[pcd|ply]" << std::endl;
  std::cout << "Optional specify the coordinate shift to translate the point cloud data by this amount.  " << std::endl;
  std::cout << "Usage: " << program_name << " cloud_filename.[pcd|ply] -x 123.4 -y 456.45 -z 89.1" << std::endl;
  std::cout << "-h:  Show this help." << std::endl;
}

template <typename PointT>
void addMinValues(const typename pcl::PointCloud<PointT>::Ptr &source_cloud,
                  typename pcl::PointCloud<PointT>::Ptr &transformed_cloud,
                  const std::vector<double> minvals)
{


    for(int i=0; i< source_cloud->size(); i++){
        pcl::PointXYZI tmp;
        tmp.x = static_cast<float>( (minvals[0] + static_cast<double> (source_cloud->points[i].x) ) );
        tmp.y = static_cast<float>( (minvals[1] + static_cast<double> (source_cloud->points[i].y) ) );
        tmp.z = static_cast<float>( (minvals[2] + static_cast<double> (source_cloud->points[i].z) ) );
        transformed_cloud->points.push_back( tmp  );

    }

    transformed_cloud->width = transformed_cloud->size();
    transformed_cloud->height = 1;


}

template <typename PointT>
void addMinValues(const typename pcl::PointCloud<PointT>::Ptr &source_cloud,
                  std::vector< std::vector<double> > &pointVector,
                  std::vector< unsigned int > &intensityVec,
                  const std::vector<double> minvals)
{


    double x,y,z;
    std::setprecision(11);

    // check if input cloud has an intensity field. If so use provided data, else create random values
    bool hasIntensity = false;
    std::string pcFields = pcl::getFieldsList(*source_cloud);
    if (pcFields.find("intensity") < std::string::npos-1)
        hasIntensity = true;


    for(int i=0; i< source_cloud->size(); i++){
        x =  (minvals[0] + static_cast<double> (source_cloud->points[i].x) ) ;
        y =  (minvals[1] + static_cast<double> (source_cloud->points[i].y) ) ;
        z =  (minvals[2] + static_cast<double> (source_cloud->points[i].z) ) ;

        if (hasIntensity)
            intensityVec.push_back( static_cast< unsigned int> (source_cloud->points[i].intensity) );
        else
            intensityVec.push_back( static_cast<unsigned int>( (rand() % 255)) );

        std::vector<double> point = {x,y,z};
        std::cerr << "DEBUG: point after adding values: \t " <<
                     point[0] << "\t" << point[1] << "\t" <<  point[2] << std::endl;
        pointVector.push_back(point);


    }

}

int toTXT( std::string file2Write,
          const std::vector< std::vector<double> > &pointVector,
          const std::vector< unsigned int > &intensityVec )
{

    std::cout << "debug: writing the point cloud to text file. This will take some time....\n";

    std::string fileToWrite = file2Write + ".txt";
    std::ofstream txtfile (fileToWrite);
   // txtfile.open(fileToWrite, std::ios::out | std::ios::binary);

    if (!txtfile.is_open() ) {
        PCL_ERROR ("Couldn't open file for writing ");
        return -1;
    }

    txtfile.precision(11);

    unsigned int intensity;

    for(int i=0; i< pointVector.size(); i++){

        intensity = intensityVec[i];
        std::vector< double > pt = pointVector.at(i);
        txtfile << pt[0] << " " << pt[1] << " " << pt[2] << " " << intensity << " " << intensity << " " << intensity << std::endl;
    }


    txtfile.close();
    std::cerr << "Saved " << pointVector.size() << " Points to " << fileToWrite << " \n" << std::endl;


}

