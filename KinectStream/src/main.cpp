////////////////////////////////////////////////////////////////////////////////////////////
//Semeplice Kinect Viewer capace di memorizzare una scena dello stream aperto quando viene premuto SPACE, il tutto in un file .pcd
//Se il programma è lanciato con l'opzione -v e nomefile.pcd, esso visualizza la nuvola di punti passata in input
///////////////////////////////////////////////////////////////////////////////////////////
//Librerie di sisema e librerie PCL
#include <iostream>
#include <pcl/io/openni_grabber.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/console/parse.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/vtk_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>

using namespace std;
using namespace pcl;
 
PointCloud<PointXYZRGBA>::Ptr cloudptr(new PointCloud<PointXYZRGBA>); // A cloud that will store colour info.
PointCloud<PointXYZ>::Ptr fallbackCloud(new PointCloud<PointXYZ>);    // A fallback cloud with just depth data.
boost::shared_ptr<visualization::CloudViewer> viewer;                 // Viewer per le nuvole di punti
boost::shared_ptr<visualization::PCLVisualizer> viewerVTK;  
Grabber* kinectGrabber;                                               // OpenNI grabber, esso si occupa di prendere i dati dal Kinect.
unsigned int filesSaved = 0;                                          // Utilizzato per la numerazione delle nuvole di punti salvate su disco.
bool saveCloud(false);                               // Variabili di controllo.
 
void printUsage(const char* programName)
{
    cout << "Utilizzo: " << programName << " [opzioni]"
         << endl
         << endl
         << "Opzioni:\n"
         << endl
         << "\t<none>     inizio sessione con Kinect device.\n"
         << "\t-h         mostra questo help.\n";
}
//Questa funzione è chiamata ogni volta che il Kinect ha dei nuovi dati da mostrare/salvare
void grabberCallback(const PointCloud<PointXYZRGBA>::ConstPtr& cloud)
{
    if (! viewer->wasStopped())
        viewer->showCloud(cloud);
 
    if (saveCloud)
    {
        stringstream stream;
        stream << "inputCloud" << filesSaved << ".pcd";
        string filename = stream.str();
        if (io::savePCDFile(filename, *cloud, true) == 0)
        {
            filesSaved++;
            cout << "Saved " << filename << "." << endl;
        }
        else PCL_ERROR("Problem saving %s.\n", filename.c_str());
 
        saveCloud = false;
    }
}
 
// Funzione per il rilevamento di avvenuta pressione di SPACE
// Se SPACE è premuto setto saveCloud a true, cosi da poter salvare in seguito la scena selezionata
void keyboardEventOccurred(const visualization::KeyboardEvent& event, void* nothing)
{
    if (event.getKeySym() == "space" && event.keyDown()) 
        saveCloud = true;
}
 
// Creo , inizializzo e ritorno un nuovo viewer.
boost::shared_ptr<visualization::CloudViewer> createViewer()
{
    boost::shared_ptr<visualization::CloudViewer> v (new visualization::CloudViewer("3D Viewer"));
    v->registerKeyboardCallback(keyboardEventOccurred);
 
    return(v);
}
 
int main(int argc, char** argv)
{
    if (console::find_argument(argc, argv, "-h") >= 0)
    {
        printUsage(argv[0]);
        return 0;
    
    } else if (argc != 1)
    		{
        		printUsage(argv[0]);
        		return 0;
    		}
   // bool justVisualize(false);
    kinectGrabber = new OpenNIGrabber();
    if (kinectGrabber == 0)
            return false;
    boost::function<void (const PointCloud<PointXYZRGBA>::ConstPtr&)> f = boost::bind(&grabberCallback, _1);
    kinectGrabber->registerCallback(f);
    viewer = createViewer();
    kinectGrabber->start();
 
    // Main loop principale
    while (! viewer->wasStopped())
        boost::this_thread::sleep(boost::posix_time::seconds(1));
 
   // if (!justVisualize)
        kinectGrabber->stop();
}
