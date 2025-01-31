////////////////////////////////////////////////////////////////////////////////////////////////////
//Questo codice esegue il filtraggio su una nuvola di punti rimuovendo il cosiddetto "rumore" ossia 
//valori (punti) appartenenti alla nuvola, ma potenzialmente scorrelati da essa. 
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
//Libreria per la rimozione del rumore
#include <pcl/filters/statistical_outlier_removal.h>

using namespace pcl;

int main (int argc, char** argv){
	//Gestione dell'input da tastiera nel caso non si abbia digitato correttamente 
	if (argc < 3)
        {
                console::print_error("Syntax: %s input.pcd -r epsilonNoise \n", argv[0]); 
		std::cout << "Con epsilonNoise si intende l'incertezza nell'assegnare i punti come scorrelati con la nuvola." << std::endl;
		std::cout << "Valori nella norma possono essere intorno a --> 1.0" << std::endl;
                return(-1);                                                         
		}													                        
        double epsilonNoise;  //Variabile contenente l'incertezza
        console::parse_argument(argc, argv, "-r", epsilonNoise);  //Assegnamento degli input alle variabili di programma

	PCLPointCloud2 cloud_blob; //Nuvola per la gestione degli errori, nel caso loadPCDFile non riesca a caricare i dati
        PointCloud<pcl::PointXYZ>::Ptr cloudBefore (new PointCloud<PointXYZ>); //Nuvola di punti contenente i dati da filtrare
        PointCloud<pcl::PointXYZ>::Ptr cloudAfter (new PointCloud<PointXYZ>); //Nuvola di punti contenente i dati filtrati
        io::loadPCDFile (argv[1], cloud_blob); //Carico da file la nuvola da filtrare 
        fromPCLPointCloud2 (cloud_blob, *cloudBefore); //Se tutto è andato bene converto la nuvola nella rappresentazione scelta

	std::cout << "Sto rimuovendo il rumore... " ;

	// Creo l'oggetto per la rimozione del rumore e setto come nuvola in input cloudBefore
	StatisticalOutlierRemoval<pcl::PointXYZ> sor;
	sor.setInputCloud (cloudBefore);
	sor.setMeanK (50); //Numero di punti vicini analizzati per ogni punto preso in considerazione
	sor.setStddevMulThresh (epsilonNoise);	//Setto il moltiplicatore di deviazione standard al valore immesso da tastiera,
						//ciò significa che tutti i punti aventi distanza > epsilonNoise 
						//saranno contrassegnati come anomali e rimossi
						
	sor.filter (*cloudAfter);//Applico il filtro e immagazzino il risultato in cloudAfter

	//Salvo la nuvola filtrata in un file chiamato noNoise_nomenuvola.pcd
	std::stringstream stream;
        stream << "noNoise_" << argv[1];
       	std::string filename = stream.str();
	io::savePCDFile<pcl::PointXYZ>(filename, *cloudAfter, true);
	std::cout << "Completato." << std::endl;
	visualization::PCLVisualizer *viewer; //Creo un oggetto di tipo PCLVisualizer per la visualizzazione dell'output
	//Dichiaro due porte rappresentanti parte destra e sinistra dello schermo (schermo --> | 1 | 2 |)
        int vPort1 = 1;		
        int vPort2 = 2;
        viewer = new visualization::PCLVisualizer (argc, argv, "3D Remove Noise"); //Istanzio l'oggetto viewer con nome 3D Remove Noise
        viewer->removePointCloud("cloudBefore", vPort1);
        viewer->removePointCloud("cloudAfter", vPort2);
        viewer->createViewPort (0.0, 0, 0.5, 1.0, vPort1); //Creo vPort1 tra margine sinistro e metà schermo
        viewer->createViewPort (0.5, 0, 1.0, 1.0, vPort2); //Creo vPort2 tra metà schermo e margine destro
	viewer->addPointCloud(cloudBefore, "cloudBefore", vPort1); //Inserisco nella vPort1 la nuvola di punti iniziale per poterla visualizzare	
	viewer->addText("CLOUD", 0.6, 0.6, "text1", vPort1); //Testo che compare nella vPort1
	viewer->addPointCloud(cloudAfter, "cloudAfter", vPort2); //Inserisco nella vPort2 la nuvola di punti filtrata per poterla visualizzare
	viewer->addText("CLOUD WITHOUT NOISE", 0.6, 0.6, "text2", vPort2); //Testo che compare nella vPort2
	viewer->setBackgroundColor (0.8275,0.8275,0.8775, 0); //imposto il colore di backgroud delle VPort
        viewer->resetCameraViewpoint();
        viewer->spin();	//Visualizzo a schermo tutto ciò che ho inserito nelle vPort in precedenza
        return (0);
} 
