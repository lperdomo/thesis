/*
 *          png2pcd_batch   -   simple command line utility to convert depth and rgb frames
 *          from png format to PCL pointcloud.
 * */


#include <pcl/io/pcd_io.h>

#include <boost/filesystem.hpp>

#include <string>
#include <vector>
#include <fstream>
#include <sstream>

using namespace pcl;
using namespace std;

void
usage ()
{
    cout    <<  "Usage: "   << endl
            <<  "dat2pcd_batch association_file" << endl
            <<  "\t\t   where association_file is a text file which should be parsed by USER implemented rule" << endl
            <<  "\t\t   for details and example of implementation check \"parse_freiburg\" function in source" << endl;
    exit(0);
}

/*
 *          This is just an example of function for parsing associating file,
 *          which should point to lidar frame.
 *          This particular function aimed for processing freiburg campus 360 dataset.
 */
bool
parse_freiburg ( string & file_name, vector<string> & dat_names, vector <string> & pcd_file_names )
{
    
    fstream assoc_file; 

    string  dat_name;

    assoc_file.open ( file_name.c_str(), std::ios::in );
    if ( ! assoc_file.is_open() )
    {
        cerr << "Can't read association file: " << file_name << endl;
        return false;
    }
    else
    {
        while( ! assoc_file.eof())
        {

            string cur_line ("");
            getline ( assoc_file, cur_line );
            stringstream magic_stream ( cur_line );

            magic_stream >> dat_name;

            if( ! cur_line.empty() )
            {
                dat_names.push_back ( dat_name );

                char tmp_str[128];
                memset ( tmp_str, 0, 128 );
                sprintf ( tmp_str, "%05d.pcd", pcd_file_names.size() );

                pcd_file_names.push_back(tmp_str);
            }
        }
    }

    cout << "Total parsed "<< pcd_file_names.size()<< " files"<< endl;	
    return true;
}

template < class T >
void
set_pixel ( T & pcl_pixel, float x, float y, float z )
{
    cerr << "set_pixel: Error - do not have proper specification for type: " << typeid(T).name() << endl;
    throw;
}

template <>
void
set_pixel ( pcl::PointXYZ & xyz_pcl_pixel, float x, float y, float z )
{
    xyz_pcl_pixel.z = z;
    xyz_pcl_pixel.x = x;
    xyz_pcl_pixel.y = y;
}

template < class T>
bool
load_cloud ( const string & file_name, PointCloud<T> & pcl_cloud )
{
     fstream dat;
     dat.open ( file_name.c_str(), std::ios::in );

     float c1, c2, c3,
        p1, p2, p3;

     pcl_cloud.is_dense = false;

     string cur_line ("");
     while (! dat.eof())
     {
        string cur_line ("");
        getline ( dat, cur_line );
        stringstream magic_stream ( cur_line );

        magic_stream >> c1 >> c2 >> c3;
        magic_stream >> p1 >> p2 >> p3;

        T   current_pixel;
        set_pixel <T> ( current_pixel, p1, p2, p3 );
        pcl_cloud.points.push_back(current_pixel);

     }

}

int
main ( int argc, char* argv[] )
{
  
    vector < string > dat_names;
    vector < string > pcd_file_names;
    
    string assoc_file = argv[1];

    if ( argc == 2 ) // it mean user pass only associated file
        parse_freiburg ( assoc_file, dat_names, pcd_file_names );


    // do we have data for continue
    if ( dat_names.empty() )
        usage();

    for ( int i = 0; i < pcd_file_names.size(); i++)
    {
        PointCloud < PointXYZ >     current_xyz_cloud;

        load_cloud < PointXYZ > ( dat_names[i], current_xyz_cloud );

        pcl::io::savePCDFileBinaryCompressed<pcl::PointXYZ> ( pcd_file_names[i], current_xyz_cloud );
    }
    return 0;
}
