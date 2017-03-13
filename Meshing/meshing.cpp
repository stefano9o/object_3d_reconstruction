////////////////////////////////////////////////////////////////////////////////////////////////////
//Codice per la ricostruzione di superfice partendo da una nuvola di punti, rappresentante 
//il totale dei dati immagazzinati con il dispositivo Kinect.
//Sotto abbiamo le inclusioni necessarie. 
/////////////////////////////////////////////////////////////////////////////////////////////////////
//Inclusioni per le operazioni di input/output
#include <pcl/io/obj_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_io.h>
//Inclusioni per ricostruzione , smussamento e visualizzazione
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/console/parse.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/concave_hull.h>
//#include <pcl/surface/vtk.h>
//#include <pcl/surface/vtk_mesh_smoothing_laplacian.h>
//#include <pcl/surface/vtk_utils.h>
#include <vtkSmartPointer.h>
#include <vtkSmoothPolyDataFilter.h>
#include <pcl/visualization/pcl_visualizer.h>

using namespace pcl;

//Funzione per la ricostruzione di superfice
boost::shared_ptr<PolygonMesh> computeConcaveHull (const PointCloud<PointXYZ>::Ptr &input, float alpha)
{
        ConcaveHull<PointXYZ> concave_hull; //Creo un oggetto di tipo ConcaveHull per la ricostruzione
        concave_hull.setInputCloud (input); //Imposto la nuvola da utilizzare in input
        concave_hull.setAlpha (alpha);	//Setto la "precisione" della ricostruzione
        boost::shared_ptr<PolygonMesh> output (new PolygonMesh); //Creo la PolygonMesh sulla quale salvo la ricostruzione
        concave_hull.reconstruct(*output);  //Applico l'algoritmo e ricostruisco la superfice
        return (output);
}

int main (int argc, char** argv)
{
	//Gestione dell'input da tastiera nel caso non si abbia digitato correttamente
        if (argc < 3)
        {
                console::print_error("Syntax: %s input.pcd -a alpha\n", argv[0]); //Alfa modifica la precisione della ricostruzione piu piccolo piu precisione
                //il problema sta nel trovare un valore medio per non incorrere in "overfitting"
																	//
		std::cout << "Alfa regola la precisione della ricostruzione, valore normale --> 0.025" << std::endl;
 				return(-1);                                                       
		}													                      
        double alpha;	//Variabile contenente la precisione
        console::parse_argument(argc, argv, "-a", alpha);	//Assegnamento degli input alle variabili di programma

	PCLPointCloud2 cloud_blob; //Nuvola per la gestione degli errori, nel caso loadPCDFile non riesca a caricare i dati
        PointCloud<PointXYZ>::Ptr cloud (new PointCloud<PointXYZ>); //Nuvola di punti contenente i dati da filtrare
        io::loadPCDFile (argv[1], cloud_blob);	//Carico da file la nuvola da filtrare 
        fromPCLPointCloud2 (cloud_blob, *cloud);//Se tutto è andato bene converto la nuvola nella rappresentazione scelta
	

        std::cout << "Effettuo Concave Hull Reconstruction...";
        boost::shared_ptr<pcl::PolygonMesh> meshIn = computeConcaveHull(cloud, alpha); //Chiamo la funzione per la ricostruzione passandogli nuvola + precisione
        std::cout << "Completato." << std::endl;

	//Qui posso salvare la ricostruzione in uno dei formati che preferisco, per poterla poi aprire in un secondo momento
	//io::saveVTKFile ("totaleSuperficie.vtk", *meshIn);
        io::saveOBJFile ("totaleSuperficieOBJ.obj", *meshIn);
	//io::savePLYFile ("totaleChairPLY.ply", *meshIn);

 	visualization::PCLVisualizer *viewer; 	//Creo un oggetto di tipo PCLVisualizer per la visualizzazione dell'output
	//Dichiaro due porte rappresentanti parte destra e sinistra dello schermo (schermo --> | 1 | 2 |)
        int vPort1 = 1;		
        int vPort2 = 2;
        viewer = new visualization::PCLVisualizer (argc, argv, "3D Meshing"); 	//Istanzio l'oggetto viewer con nome 3D Meshing
        viewer->removePointCloud("cloud", vPort1);
        viewer->removePolygonMesh("meshIn", vPort2);
        viewer->createViewPort (0.0, 0, 0.5, 1.0, vPort1);	//Creo vPort1 tra margine sinistro e metà schermo
        viewer->createViewPort (0.5, 0, 1.0, 1.0, vPort2);	//Creo vPort2 tra metà schermo e margine destro
	viewer->addPointCloud(cloud, "cloud", vPort1);		//Inserisco nella vPort1 la nuvola di punti iniziale per poterla visualizzare	
	viewer->addText("CLOUD", 0.6, 0.6, "text1", vPort1);	//Testo che compare nella vPort1
	viewer->addPolygonMesh(*meshIn, "meshIn", vPort2);	//Inserisco nella vPort2 la nuvola di punti filtrata per poterla visualizzare
	viewer->addText("CLOUD MESHED", 0.6, 0.6, "text2", vPort2); //Testo che compare nella vPort2
        viewer->setBackgroundColor (0.8275,0.8275,0.8775, 0); //imposto il colore di backgroud delle VPort
        viewer->resetCameraViewpoint();
        viewer->spin();	//Visualizzo a schermo tutto ciò che ho inserito nelle vPort in precedenza
        return (0);

} 
