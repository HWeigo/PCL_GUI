#include "BaseVisualization.h"

BaseVisualization::BaseVisualization(boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_in, string id_in) : 
	viewer(viewer_in), id(id_in)
{
}


BaseVisualization::~BaseVisualization()
{
}
