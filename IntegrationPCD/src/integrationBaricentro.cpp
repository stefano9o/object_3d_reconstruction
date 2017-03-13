#include <boost/make_shared.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/point_representation.h>
#include <Eigen/Core>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>
#include <pcl/features/normal_3d.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>

using pcl::visualization::PointCloudColorHandlerGenericField;
using pcl::visualization::PointCloudColorHandlerCustom;

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointNormal PointNormalT;
typedef pcl::PointCloud<PointNormalT> PointCloudWithNormals;

pcl::visualization::PCLVisualizer *p;
//viewport destro e sinistro utilizzate nella visulizzazione
int vp_1, vp_2;

//Creazione di una nuova struttura per una migliore gestione delle pointcloud
struct PCD
{
  PointCloud::Ptr cloud;
  std::string f_name;

  PCD() : cloud (new PointCloud) {};
};

struct PCDComparator
{
  bool operator () (const PCD& p1, const PCD& p2)
  {
    return (p1.f_name < p2.f_name);
  }
};

// Definisco una nuova rappresentazione del punto aggiungendo la curvatura < x, y, z, curvature >
class MyPointRepresentation : public pcl::PointRepresentation <PointNormalT>
{
  using pcl::PointRepresentation<PointNormalT>::nr_dimensions_;
public:
  MyPointRepresentation (){	nr_dimensions_ = 4;	}

  // Ridefinizione del metodo copyToFloatArray
  virtual void copyToFloatArray (const PointNormalT &p, float * out) const
  {
    // < x, y, z, curvature >
    out[0] = p.x;
    out[1] = p.y;
    out[2] = p.z;
    out[3] = p.curvature;
  }
};

////////////////////////////////////////////////////////
//Visualizzo source e target nelle rispettive viewports 
////////////////////////////////////////////////////////
void showCloudsLeft(const PointCloud::Ptr cloud_target, const PointCloud::Ptr cloud_source)
{
  p->removePointCloud ("vp1_target");
  p->removePointCloud ("vp1_source");

  PointCloudColorHandlerCustom<PointT> tgt_h (cloud_target, 0, 255, 0);
  PointCloudColorHandlerCustom<PointT> src_h (cloud_source, 255, 0, 0);
  p->addPointCloud (cloud_target, tgt_h, "vp1_target", vp_1);
  p->addPointCloud (cloud_source, src_h, "vp1_source", vp_1);

  PCL_INFO ("Press q to begin the registration.\n");
  p-> spin();
}

void showCloudsRight(const PointCloudWithNormals::Ptr cloud_target, const PointCloudWithNormals::Ptr cloud_source)
{
  p->removePointCloud ("source");
  p->removePointCloud ("target");


  PointCloudColorHandlerGenericField<PointNormalT> tgt_color_handler (cloud_target, "curvature");
  if (!tgt_color_handler.isCapable ())
      PCL_WARN ("Cannot create curvature color handler!");

  PointCloudColorHandlerGenericField<PointNormalT> src_color_handler (cloud_source, "curvature");
  if (!src_color_handler.isCapable ())
      PCL_WARN ("Cannot create curvature color handler!");


  p->addPointCloud (cloud_target, tgt_color_handler, "target", vp_2);
  p->addPointCloud (cloud_source, src_color_handler, "source", vp_2);

  p->spinOnce();
}

////////////////////////////////////////////////////////////////////////////////
/** \Carica un set di file .pcd in attesa dell'integrazione
  * \parametro argc contiene il numero di argomenti (passati da main ())
  * \parametro argv contiene l'attuale comando eseguito da bash (passati da main ())
  * \parametro models contiene il vettore di pointcloud da integrare
  */
void loadData (int argc, char **argv, std::vector<PCD, Eigen::aligned_allocator<PCD> > &models)
{
  std::string extension (".pcd");
  for (int i = 1; i < argc; i++)
  {
    std::string fname = std::string (argv[i]);
    //Controllo che la dimensione della stringa sia maggiore dell'estensione
    if (fname.size () <= extension.size ())
      continue;

    std::transform (fname.begin (), fname.end (), fname.begin (), (int(*)(int))tolower);

    //controllo se file è effettivamente .pcd
    if (fname.compare (fname.size () - extension.size (), extension.size (), extension) == 0)
    {
      // Carico la nuvola e la aggiungo al vettore models
      PCD m;
      m.f_name = argv[i];
      pcl::io::loadPCDFile (argv[i], *m.cloud);
      //rimuovo i punti NAN dalla nuvola
      std::vector<int> indices;
      pcl::removeNaNFromPointCloud(*m.cloud,*m.cloud, indices);

      models.push_back (m);
    }
  }
}
////////////////////////////////////////////////////////////////////////////////
/** \Allineamento di una coppia di pointcloud e ritorno del risultato
  * \parametro cloud_src contiene source PointCloud
  * \parametro cloud_tgt contiene target PointCloud
  * \parametro output contiene l'allineamento di source su target
*/////////////////////////////////////////////////////////////////////////
void pairAlign (const PointCloud::Ptr cloud_src, const PointCloud::Ptr cloud_tgt, PointCloud::Ptr output, bool downsample = false)
{
  //Si effettua una riduzione dei punti in modo da aumentare la velocità di elaborazione mantenendo la consistenza
  //Viene effettuata se la nuvola di punti è abbastanza grande 
  PointCloud::Ptr src (new PointCloud);
  PointCloud::Ptr tgt (new PointCloud);
  pcl::VoxelGrid<PointT> grid;
  if (downsample) //se a true diminuisce il numero di punti
  {
    grid.setLeafSize (0.05, 0.05, 0.05);
    grid.setInputCloud (cloud_src);
    grid.filter (*src);

    grid.setInputCloud (cloud_tgt);
    grid.filter (*tgt);
  }
  else
  {
    src = cloud_src;
    tgt = cloud_tgt;
  }
   Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity ();
   //Calcolo il baricentro della nuvola di punti per oterla ruotare attorno al suo asse principale
   float tempX = 0, tempY = 0, tempZ = 0;
   float middleX, middleY, middleZ;

	for(size_t k = 0; k < src->points.size (); ++k){
   	  	tempX = tempX + src->points[k].x;
	      	tempY = tempY + src->points[k].y;
	      	tempZ = tempZ + src->points[k].z;
	}
	middleX = tempX / src->points.size ();
    middleY = tempY / src->points.size ();
    middleZ = tempZ / src->points.size ();

    //Pre-ruoto la matrice source di circa 45 gradi per effettuare meno allineamenti e meno iterazioni

	Eigen::Matrix4f zeroT;
			zeroT <<      1, 0, 0, -middleX,
 		      		      0, 1, 0, -middleY,
  				      0, 0, 1, -middleZ,
   			              0, 0, 0, 	     1;
	Eigen::Matrix4f degree45;
			//AsseY
  			degree45 <<   cos(-45), 0, sin(-45),       0,
				      0, 1,        0,       0,
 		      	              -sin(-45), 0, cos(-45),       0,
  				      0, 0,        0,       1;
 	Eigen::Matrix4f tZero;
			tZero <<      1, 0, 0, middleX,
 		      		      0, 1, 0, middleY,
  				      0, 0, 1, middleZ,
   			              0, 0, 0, 		 1;
  
  pcl::transformPointCloud (*src, *src, zeroT); 
  pcl::transformPointCloud (*src, *src, degree45);
  pcl::transformPointCloud (*src, *src, tZero);
  Ti = zeroT * Ti;
  Ti = degree45 * Ti;
  Ti = tZero * Ti;

  //Calcolo le normali e curvatura della superfice 
  PointCloudWithNormals::Ptr points_with_normals_src (new PointCloudWithNormals);
  PointCloudWithNormals::Ptr points_with_normals_tgt (new PointCloudWithNormals);

  pcl::NormalEstimation<PointT, PointNormalT> norm_est;
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
  norm_est.setSearchMethod (tree);
  norm_est.setKSearch (100);
  
  norm_est.setInputCloud (src);
  norm_est.compute (*points_with_normals_src);
  pcl::copyPointCloud (*src, *points_with_normals_src);

  norm_est.setInputCloud (tgt);
  norm_est.compute (*points_with_normals_tgt);
  pcl::copyPointCloud (*tgt, *points_with_normals_tgt);

  //Istanzio un oggetto MyPointRepresentation e inizializzo i dati per avere una rappresentazione bilanciata
  MyPointRepresentation point_representation;
  float alpha[4] = {1.0, 1.0, 1.0, 1.0};
  point_representation.setRescaleValues (alpha);

  // ALLINEAMENTO
  pcl::IterativeClosestPointNonLinear<PointNormalT, PointNormalT> reg;
  reg.setTransformationEpsilon (1e-6);
  //Imposto la distanza massima tra le due nuvole src e tgt, in questo caso sarà 10cm
  //Nota: questo parametro sarà da regolare in base al dataset utilizzato
  reg.setMaxCorrespondenceDistance (0.1);  
  //Imposto la rappresentazione del punto
  reg.setPointRepresentation (boost::make_shared<const MyPointRepresentation> (point_representation));

  reg.setInputCloud (points_with_normals_src);
  reg.setInputTarget (points_with_normals_tgt);

  //Eseguo l'allineamento in un loop e visualizzo i risultati
  Eigen::Matrix4f prev;
  Eigen::Matrix4f targetToSource;
  PointCloudWithNormals::Ptr reg_result = points_with_normals_src;
  reg.setMaximumIterations (2);
  //Aumentando le iterazioni divento molto piu preciso nell'allineare gli oggetti ma peggioro in termini di tempo
  for (int i = 0; i < 150; ++i) 
  {
    PCL_INFO ("Iterazione Nr. %d.\n", i);

    // salvo la nuvola ottenuta per poterla visualizzare in seguito
    points_with_normals_src = reg_result;
    // Imposto la nuvola da allineare e procedo con l'allineamento vero e proprio
    reg.setInputCloud (points_with_normals_src);
    reg.align (*reg_result);
	// Accumulo in una sola le trasformazioni tra ogni iterazione di allineamento
    Ti = reg.getFinalTransformation () * Ti;	
	// Se la differenza tra questa trasformazione e la precedente è piccola rispetto a una soglia
	// raffiniamo il processo riducendo la distanza massima tra le due nuvole di punti (MaxCorrespondenceDistance) 
    if (fabs ((reg.getLastIncrementalTransformation () - prev).sum ()) < reg.getTransformationEpsilon ())
      	reg.setMaxCorrespondenceDistance (reg.getMaxCorrespondenceDistance () - 0.001);
    
    prev = reg.getLastIncrementalTransformation ();

    // Mostro a video lo stato corrente dell'allineamento
    showCloudsRight(points_with_normals_tgt, points_with_normals_src);
  }
  std::cout <<"Trasformation Source-Target Matrix\n"<< Ti << endl;
  //Ottengo la trasformazione inversa da target a source
  targetToSource = Ti.inverse();	
  // Applico la trasformazione trovata a cloud_tgt in modo che si allinei con precisione nell'output 
  pcl::transformPointCloud (*cloud_tgt, *output, targetToSource);

  p->removePointCloud ("source");
  p->removePointCloud ("target");

  PointCloudColorHandlerCustom<PointT> cloud_tgt_h (output, 0, 255, 0);    //Coloro di verde il target
  PointCloudColorHandlerCustom<PointT> cloud_src_h (cloud_src, 255, 0, 0); //Coloro di rosso la source
  p->addPointCloud (output, cloud_tgt_h, "target", vp_2);
  p->addPointCloud (cloud_src, cloud_src_h, "source", vp_2);

  PCL_INFO ("Premi q per continuare la registrazione.\n");
  p->spin ();

  p->removePointCloud ("source"); 
  p->removePointCloud ("target");

  // Aggiungo la sorgente ruotata all'insieme dei punti già allineati
  *output += *cloud_src;
 }


int main (int argc, char** argv)
{
  //Caricamento dei dati
  std::vector<PCD, Eigen::aligned_allocator<PCD> > data;
  loadData (argc, argv, data);

  //Gestione dell'input dato dall'utente
  if (data.empty ())
  {
    PCL_ERROR ("Syntax is: %s <source.pcd> <target.pcd> [*]", argv[0]);
    PCL_ERROR ("[*] - file multipli possono essere dati in input inserendoli tra parentesi  (es. prova[1-8].pcd)");
	PCL_ERROR ("La prima nuvola è allineata alla seconda , la seconda alla terza e così via fino all'esaurirsi delle nuvole.");
    return (-1);
  }
  PCL_INFO ("Datasets %d caricati.", (int)data.size ());
  
  // Creo un oggetto PCLVisualizer per la visualizzazione a video delle nuvole di punti
  p = new pcl::visualization::PCLVisualizer (argc, argv, "Integration PCD");
  p->createViewPort (0.0, 0, 0.5, 1.0, vp_1);
  p->createViewPort (0.5, 0, 1.0, 1.0, vp_2);

  PointCloud::Ptr source, target;
  PointCloud::Ptr temp (new PointCloud);
  // All'interno di questo for seleziono in ordine le varie nuvole per le traformazioni
  for (size_t i = 1; i < data.size (); ++i)
  {
	if( i >= 2 ){	
			source = data[i].cloud;
			target = temp;
	}else {
			source = data[i].cloud;
 			target = data[i-1].cloud;
	}

    //Mostro a video source e target rispettivamente sulla destra e sulla sinistra dello schermo
    showCloudsLeft(source, target);
	
    PCL_INFO ("Aligning %s (%d) with %s (%d).\n", data[i-1].f_name.c_str (), source->points.size (), data[i].f_name.c_str (), target->points.size ());
    pairAlign (source, target, temp, true);

  }
   std::stringstream ssTOT;
   ssTOT << "totale.pcd";
   pcl::io::savePCDFile (ssTOT.str (), *temp, true);
}
