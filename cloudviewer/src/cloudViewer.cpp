////////////////////////////////////////////////////////////////////////////////////////////////////
//Semplice visualizatore di nuvole di punti passate in input da terminale
//Sotto abbiamo le inclusioni necessarie. 
/////////////////////////////////////////////////////////////////////////////////////////////////////
//Librerie di sistema e librerie PCL
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/console/parse.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>

using namespace pcl;

int main (int argc, char** argv){
	//Gestione dell'input da tastiera nel caso non si abbia digitato correttamente 
	if (argc < 1)
        {
                console::print_error("Syntax: %s input.pcd  \n", argv[0]); 
                return(-1);                                                         
		}													                        

        PCLPointCloud2 cloud_blob; //Nuvola per la gestione degli errori, nel caso loadPCDFile non riesca a caricare i dati
        PointCloud<PointXYZ>::Ptr cloudBefore (new PointCloud<PointXYZ>); //Nuvola di punti contenente i dati da filtrare
        PointCloud<PointXYZ>::Ptr cloudAfter (new PointCloud<PointXYZ>); //Nuvola di punti contenente i dati filtrati
        io::loadPCDFile (argv[1], cloud_blob);	//Carico da file la nuvola da filtrare 
        fromPCLPointCloud2 (cloud_blob, *cloudBefore);//Se tutto è andato bene converto la nuvola nella rappresentazione scelta

	visualization::PCLVisualizer *viewer; //Creo un oggetto di tipo PCLVisualizer per la visualizzazione dell'output
        int vPort1 = 1;		
        viewer = new visualization::PCLVisualizer (argc, argv, "3D Visualizer"); //Istanzio l'oggetto viewer
        viewer->removePointCloud("cloud", vPort1);
        viewer->createViewPort (0.0, 0, 1.0, 1.0, vPort1);
        //Creo vPort1 tra margine sinistro e margine destro
	viewer->addPointCloud(cloudBefore, "cloud", vPort1);	//Inserisco nella vPort1 la nuvola di punti iniziale per poterla visualizzare	
	viewer->addText("CLOUD", 1.0, 1.0, "text1", vPort1); //Testo che compare nella vPort1
	viewer->setBackgroundColor(0.8275,0.8275,0.8775);
        viewer->resetCameraViewpoint();
        viewer->spin();	//Visualizzo a schermo tutto ciò che ho inserito nelle vPort in precedenza
        return (0);
} 
