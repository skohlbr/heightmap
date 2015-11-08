#include <heightmap/image.hpp>

#include <ros/console.h>
#include <IL/il.h>

using namespace heightmap;

static bool devilInitialized = false;

void ensureDevilInit()
{
	if (!devilInitialized) {
		ilInit();
		devilInitialized = true;
	}
}

Image::Image() : loaded_(false), handle_(0) { }
Image::Image(ILuint handle) : loaded_(true), handle_(handle) { }
Image::~Image()
{
	if (loaded_)
		ilDeleteImages(1, &handle_);
}

Image Image::loadFile(const std::string &filename)
{
	ILuint img;
	ILint n_rows, n_cols;
	ILdouble *data;

	ensureDevilInit();
	ilGenImages(1, &img);
	ilBindImage(img);

	if (ilLoadImage(filename.c_str()) != IL_TRUE) {
		ROS_ERROR("%s: couldn't load\n", filename.c_str());
		return Image();
	}

	// Let DevIL do the hard work for us
	if (ilConvertImage(IL_LUMINANCE, IL_DOUBLE) != IL_TRUE) {
		ROS_ERROR("%s: couldn't convert to single-channel float: %s\n",
		          filename.c_str(), ilGetString(ilGetError()));
		return Image();
	}

	return Image(img);
}

MatrixRef<double*> Image::data()
{
	if (isNull())
		return {};  // Return a null MatrixRef

	ilBindImage(handle_);
	double *data = (double*) ilGetData();
	unsigned int n_rows = ilGetInteger(IL_IMAGE_HEIGHT);
	unsigned int n_cols = ilGetInteger(IL_IMAGE_WIDTH);
	return  MatrixRef<double*>(data, n_rows, n_cols);
}
