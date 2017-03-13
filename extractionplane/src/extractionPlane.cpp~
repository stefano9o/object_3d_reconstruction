/////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Questo codice esegue il filtraggio su una nuvola di punti estraendo da essa il piano piu grande rilevato
//(in questo caso stiamo parlando del pavimento), la variabile epsilon viene utilizzata per includere
//più o meno punti da estrarre a seconda della loro vicinanza con il piano. 
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
//Librerie per la rimozione di piani
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h> 
#include <pcl/filters/extract_indices.h>

using namespace pcl;

int main (int argc, char** argv){
	
	//Gestione dell'input da tastiera nel caso non si abbia digitato correttamente 
	if (argc < 3)
        {
                console::print_error("Syntax: %s input.pcd -e epsilon\n", argv[0]);
		std::cout << "Con epsilon si intende l'incertezza nell'assegnare i punti come appartenenti al piano." << std::endl;
		std::cout << "Valori nella norma possono essere intorno a --> 0.025" << std::endl;
                return(-1);                                                         
		}													                        
        double epsilon;	//Variabile contenente il range di inclusione dei punti nel piano
        console::parse_argument(argc, argv, "-e", epsilon); //Assegnamento degli input alle variabili di programma

	PCLPointCloud2 cloud_blob; //Nuvola per la gestione degli errori, nel caso loadPCDFile non riesca a caricare i dati
        PointCloud<PointXYZ>::Ptr cloudBefore (new PointCloud<PointXYZ>); //Nuvola di punti contenente i dati da filtrare
        PointCloud<PointXYZ>::Ptr cloudAfter (new PointCloud<PointXYZ>); //Nuvola di punti contenente i dati filtrati
        io::loadPCDFile (argv[1], cloud_blob);	//Carico da file la nuvola da filtrare 
        fromPCLPointCloud2 (cloud_blob, *cloudBefore);	//Se tutto è andato bene converto la nuvola nella rappresentazione scelta
        

	std::cout << "Sto filtrando... " <<std::endl;;
	std::cerr << "Dimensione della nuvola prima del filtraggio: " << cloudBefore->width * cloudBefore->height << " punti" << std::endl;

	ModelCoefficients::Ptr coefficients (new ModelCoefficients ());	//Conterrà i coefficenti che individuano univocamente il piano
	PointIndices::Ptr inliers (new PointIndices ()); //Conterrà i punti appartenenti al piano
	SACSegmentation<PointXYZ> seg;	//Creo l'oggetto utilizzato per la stima del piano
        seg.setOptimizeCoefficients (true); //E' opzionale, ma ottimizza il calcolo dei coefficenti
	seg.setModelType (SACMODEL_PLANE);  //Seleziono il modello di riferimento del piano
	seg.setMethodType (SAC_RANSAC);	//Seleziono il metodo per la stima del piano
	seg.setMaxIterations (1000);
	seg.setDistanceThreshold (epsilon); //Setto l'incertezza con cui considero i punti
	ExtractIndices<PointXYZ> extract; //Creo l'oggetto filtro per l'estrazione del piano
	seg.setInputCloud (cloudBefore);
	seg.segment (*inliers, *coefficients);//Inserisco in inliers tutti i punti che appartengono
					     //al piano sulla base dei coefficenti calcolati da RANSAC
					     
	if (inliers->indices.size () == 0){ //Se la grandezza di inliers è zero non ho punti nel piano
		std::cerr << "Non è possibile effettuare una stima di un piano con il dataset fornito." << std::endl;
		exit(0);
	}
	//Procedo adesso all'estrazione dei punti appartenenti al piano
	extract.setInputCloud (cloudBefore);
	extract.setIndices (inliers);
	extract.setNegative (true); //L'oggetto filtro manterrà i punti fuori dal piano e scarterà gli altri(Mettendo false faccio l'opposto)
	 extract.filter (*cloudAfter);	//Applico il filtro e immagazzino il risultato in cloudAfter
	std::cerr << "Dimensione del piano: " << cloudAfter->width * cloudAfter->height << " punti" << std::endl;

	//Salvo la nuvola filtrata in un file chiamato noplane_nomenuvola.pcd
	std::stringstream stream;
        stream << "noPlane_" << argv[1];
       	std::string filename = stream.str();
	io::savePCDFile<PointXYZ>(filename, *cloudAfter, true);
	std::cout << "Completato." << std::endl;
		
        visualization::PCLVisualizer *viewer; //Creo un oggetto di tipo PCLVisualizer per la visualizzazione dell'output
	//Dichiaro due porte rappresentanti parte destra e sinistra dello schermo (schermo --> | 1 | 2 |)
        int vPort1 = 1;		
        int vPort2 = 2;
        viewer = new visualization::PCLVisualizer (argc, argv, "3D Viewer Filtraggio");	//Istanzio l'oggetto viewer con nome 3D Viewer Filtraggio
        viewer->removePointCloud("cloudBefore", vPort1);
        viewer->removePointCloud("cloudAfter", vPort2);
        viewer->createViewPort (0.0, 0, 0.5, 1.0, vPort1);	//Creo vPort1 tra margine sinistro e metà schermo
        viewer->createViewPort (0.5, 0, 1.0, 1.0, vPort2);	//Creo vPort2 tra metà schermo e margine destro
		viewer->addPointCloud(cloudBefore, "cloudBefore", vPort1);//Inserisco nella vPort1 la nuvola di punti iniziale per poterla visualizzare	
	viewer->addText("CLOUD", 0.6, 0.6, "text1", vPort1);	//Testo che compare nella vPort1
	viewer->addPointCloud(cloudAfter, "cloudAfter", vPort2); //Inserisco nella vPort2 la nuvola di punti filtrata per poterla visualizzare
	viewer->addText("CLOUD WITHOUT PLANE", 0.6, 0.6, "text2", vPort2);//Testo che compare nella vPort2
	viewer->setBackgroundColor (0.8275,0.8275,0.8775, 0); //imposto il colore di backgroud delle VPort
        viewer->resetCameraViewpoint();
        viewer->spin();	//Visualizzo a schermo tutto ciò che ho inserito nelle vPort in precedenza
        return (0);
} 
