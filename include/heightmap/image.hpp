#include <string>
#include <heightmap/sparse_block_matrix.hpp>

namespace heightmap
{
	/// This module exists to abstract away the library used for
	/// image loading/decoding, so we can easily switch to another
	/// library in the future.



	struct Image {
	public:
		// Load the image file whose name is `filename'. The image is
		// supposed to have only one channel. Returns a ImgHandle
		// corresponding to the newly created image.  If anything during
		// the process fails, a message describing the error is printed
		// (through ROS_* macros), and a null ImgHandle is returned.
		static Image loadFile(const std::string &filename);

		// Create a new single-channel image from existing luminance data.
		static Image fromData(MatrixRef<double*> data);

		~Image();

		/// Returns a MatrixRef pointing to the loaded image's data,
		/// or a null MatrixRef if this is a null Image handle.
		MatrixRef<double*> data();

		inline bool isNull() const { return ! loaded_; }

	private:
		// Just a forward declaration, so we don't include the whole header
		typedef unsigned int ILuint;

		Image();
		Image(ILuint);

		bool loaded_;
		ILuint handle_;
	};

}
