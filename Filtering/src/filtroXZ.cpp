/////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Questo codice esegue il filtraggio su una nuvola di punti lungo gli assi X e Z 
//con intervalli inseriti da tastiera e ne visualizza l'output a video utilizzando la librerie pcl dedicate.
//Sotto abbiamo le inclusioni necessarie. 
/////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Librerie di sistema e librerie PCL
#include <iostream>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/console/parse.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
//Libreria per il filtraggio
#include <pcl/filters/passthrough.h>

using namespace pcl;

int main (int argc, char** argv){
	//Gestione dell'input da tastiera nel caso non si abbia digitato correttamente 
	if (argc < 3)
        {
                console::print_error("Syntax: %s input.pcd -f intervalFilteringZ,intervalFilteringX\n", argv[0]); 
		std::cout << "Filtro sulla Z tra : [0.0, intervalFilteringZ]" << std::endl;
		std::cout << "Filtro sulla X tra : [-intervalFilteringX, intervalFilteringX]" << std::endl;
                return(-1);                                                         
		}													                        
        double rangeZ,rangeX; //Variabili contenenti i range di filtraggio su X e Z
	console::parse_2x_arguments(argc, argv, "-f", rangeZ, rangeX); 	//Assegnamento degli input alle variabili di programma

	PCLPointCloud2 cloud_blob; //Nuvola per la gestione degli errori, nel caso loadPCDFile non riesca a caricare i dati
        PointCloud<PointXYZ>::Ptr cloudBefore (new PointCloud<PointXYZ>); //Nuvola di punti contenente i dati da filtrare
        PointCloud<PointXYZ>::Ptr cloudAfter (new PointCloud<PointXYZ>); //Nuvola di punti contenente i dati filtrati
        io::loadPCDFile (argv[1], cloud_blob);	//Carico da file la nuvola da filtrare 
        fromPCLPointCloud2 (cloud_blob, *cloudBefore);//Se tutto è andato bene converto la nuvola nella rappresentazione scelta

	std::cerr << "Sto filtrando... " ;
	std::cerr << "sulla Z ... " ;
	//Creo l'oggetto filtro sulla Z, con nome 'z', settando in input cloudBefore e come range di filtraggio tra 0 e rangeZ
	PassThrough<PointXYZ> passZ;
  	passZ.setInputCloud (cloudBefore);
  	passZ.setFilterFieldName ("z");
  	passZ.setFilterLimits (0.0, rangeZ); 
  	passZ.filter (*cloudAfter); //Applico il filtro e immagazzino il risultato in cloudAfter
	std::cerr << "sulla X ... " ;
	//Creo l'oggetto filtro sulla X, con nome 'x', settando in input cloudBefore e come range di filtraggio tra -rangeX e rangeX
	PassThrough<PointXYZ> passX;
  	passX.setInputCloud (cloudAfter);
  	passX.setFilterFieldName ("x");
  	passX.setFilterLimits (-rangeX,rangeX);
  	passX.filter (*cloudAfter); //Applico il filtro e immagazzino il risultato in cloudAfter
					
	//Salvo la nuvola filtrata in un file chiamato filter_nomenuvola.pcd
	std::stringstream stream;
        stream << "filter_" << argv[1];
       	std::string filename = stream.str();
	io::savePCDFile<PointXYZ>(filename, *cloudAfter, true);
	std::cout << "Completato." << std::endl;

        visualization::PCLVisualizer *viewer; //Creo un oggetto di tipo PCLVisualizer per la visualizzazione dell'output
	//Dichiaro due porte rappresentanti parte destra e sinistra dello schermo (schermo --> | 1 | 2 |)
        int vPort1 = 1;		
        int vPort2 = 2;
        viewer = new visualization::PCLVisualizer (argc, argv, "3D Viewer Filtraggio"); //Istanzio l'oggetto viewer con nome 3D Viewer Filtraggio
        viewer->removePointCloud("cloudBefore", vPort1);
        viewer->removePointCloud("cloudAfter", vPort2);
        viewer->createViewPort (0.0, 0, 0.5, 1.0, vPort1); //Creo vPort1 tra margine sinistro e metà schermo
        viewer->createViewPort (0.5, 0, 1.0, 1.0, vPort2); //Creo vPort2 tra metà schermo e margine destro
	viewer->addPointCloud(cloudBefore, "cloudBefore", vPort1); //Inserisco nella vPort1 la nuvola di punti iniziale per poterla visualizzare	
	viewer->addText("CLOUD", 0.6, 0.6, "text1", vPort1); //Testo che compare nella vPort1
	viewer->addPointCloud(cloudAfter, "cloudAfter", vPort2); //Inserisco nella vPort2 la nuvola di punti filtrata per poterla visualizzare
	viewer->addText("CLOUD FILTERED", 0.6, 0.6, "text2", vPort2); //Testo che compare nella vPort2
	viewer->setBackgroundColor (0.8275,0.8275,0.8775, 0); //imposto il colore di backgroud delle VPort
        viewer->resetCameraViewpoint();
        viewer->spin();	//Visualizzo a schermo tutto ciò che ho inserito nelle vPort in precedenza
        return (0);
} 
