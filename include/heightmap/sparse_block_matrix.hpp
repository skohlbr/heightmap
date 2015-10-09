#pragma once

#include <unordered_map>
#include <utility>
#include <memory>
#include <cassert>
#include <cmath>

#include <gtest/gtest.h>

#include <boost/shared_array.hpp>
#include <boost/functional/hash.hpp>

namespace heightmap
{

	/// An Index is basically a fixed-size array of numbers (a tuple)
	/// with arithmetic operations defined on it. Index objects of the
	/// same type (element type and number must match) can be added,
	/// subtracted and multiplied together element-wise, just like
	/// tuples in linear algebra.  They can also be create with the
	/// uniform initialization syntax:
	///		Index<int, 2> idx {3, 4};
	/// Lastly, `Index<>` means the default `Index<int, 2>` type
	/// useful for the 2-dimensional matrices used in the rest.
	template<typename T = int, std::size_t N = 2>
	struct Index : public std::array<T, N> {
		Index(std::initializer_list<T> init_list) {
			assert (init_list.size() == N);
			int i = 0;
			for (const T& item : init_list)
				(*this)[i++] = item;
		}

		friend Index operator+(const Index& lhs, const Index& rhs) {
			Index ret = lhs;
			for(int i=0; i < N; i++)
				ret[i] += rhs[i];
			return ret;
		}

		friend Index operator-(const Index& lhs, const Index& rhs) {
			Index ret = lhs;
			for(int i=0; i < N; i++)
				ret[i] -= rhs[i];
			return ret;
		}

		friend Index operator-(const Index& op) {
			Index ret = op;
			for(int i=0; i < N; i++)
				ret[i] = -ret[i];
			return ret;
		}

		friend Index operator*(const Index& lhs, const Index& rhs) {
			Index ret = lhs;
			for(int i=0; i < N; i++)
				ret[i] *= rhs[i];
			return ret;
		}
	};

	/// Region<T, N> objects denote a rectangular regions in an
	/// N-dimensional matrix with indices of type T. The region is
	/// represented as an offset and size (both expressed in number of
	/// elements). Regions can be created with the uniform
	/// initialization syntax, with the offset and the size as
	/// arguments:
	///		Region<> region { {3,2}, {4,5} };
	template <typename T = int, size_t N = 2>
	struct Region {
		typedef T ElementT;

		Region(Index<T,N> offset, Index<T,N> size)
			: offset(offset), size(size)
			{
				for (int i=0; i < N; i++) {
					if (size[i] < 0) {
						size[i] = -size[i];
						offset[i] -= size[i];
					}
				}
			}

		const Index<T, N> offset, size;

		inline bool contains(Index<T,N> point) const {
			return (point[0] >= offset[0] &&
			        point[0] < offset[0]+size[0] &&
			        point[1] >= offset[1] &&
			        point[1] < offset[1]+size[1]);
		}
	};

	/// A MatrixRef gives the structure of a matrix to a raw, flat
	/// array of numbers.  The array is accessed through a pointer of
	/// type ElementPtr, given by the user.
	///
	/// The template automatically adapts to the given pointer type:
	/// it uses a pointer of that type to look at the matrix, and it
	/// infers what is the data type of each element, (you can use
	/// `MatrixRef::ElementT` to refer to this data type). This allows
	/// the user to manage the array in a variety of ways. For
	/// example, just by using shared_ptr as underlying pointer type
	/// allows to manage the array memory automatically through
	/// reference counting. (`MatrixRef<std::shared_ptr>` is already
	/// available as the alias simply named `Matrix`).
	///
	/// While direct access can be performed with the subscript
	/// operator, "block references" are preferred when operations
	/// must be performed on a region of the matrix. A block reference
	/// to a part of this matrix can be obtained through the `cut` and
	/// `cutConst` methods. The SliceElementPtr template argument
	/// determines the pointer type for their result (you should
	/// choose an appropriate type to ensure that the array memory is
	/// properly managed; shared_ptr already works).
	template <typename ElementPtr,
	          typename SliceElementPtr = ElementPtr>
	class MatrixRef {
	public:
		typedef ElementPtr PtrT;
		typedef typename std::pointer_traits<PtrT>::element_type ElementT;
		typedef MatrixRef<SliceElementPtr> Slice;
		typedef typename std::pointer_traits<PtrT>::template rebind<const ElementT> ConstSlicePtr;
		typedef MatrixRef<ConstSlicePtr> ConstSlice;

		/// Default constructor: construct a "null" MatrixRef.
		/// (Access to this matrix's data will fail, usually with
		/// a std::range_error exception)
		MatrixRef()
			: n_rows_(0), n_cols_(0),
			  offset_(0), row_stride_(0), ptr_()
			{}

		/// Construct a MatrixRef.
		/// Args:
		///   `ptr`:
		///      pointer to the data block
		///   `nRows`, `nCols`:
		///      size of the matrix
		///   `offset`:
		///      number of elements to skip, beginning from ptr, to
		///      get to the start of the memory region
		///	  `rowStride`:
		///	     number of elements to skip between successive rows.
		MatrixRef(PtrT ptr,
		          unsigned nRows, unsigned nCols,
		          unsigned offset = 0, unsigned rowStride = 0)
			: n_rows_(nRows), n_cols_(nCols),
			  offset_(offset), row_stride_(rowStride), ptr_(ptr)
			{}

		/// Allocate a matrix with the given size and construct a
		/// MatrixRef that "sees" it
		static MatrixRef<ElementPtr, SliceElementPtr>
		create(unsigned nRows, unsigned nCols)
			{
				return {ElementPtr{new ElementT[nRows * nCols]},
						nRows, nCols, 0, 0};
			}

		inline bool isNull() const { return (ptr_ == nullptr); }
		inline int rows() const { return n_rows_; }
		inline int cols() const { return n_cols_; }
		inline Index<> size() const { return {n_rows_, n_cols_}; }

		inline ElementT& operator[](Index<> index) {
			return ptr_[ flatIndex(index[0], index[1]) ];
		}
		inline const ElementT& operator[](Index<> index) const {
			return ptr_[ flatIndex(index[0], index[1]) ];
		}

		/// Set all elements in this matrix to the given value.  (Note
		/// that if this MatrixRef is a slice of a bigger matrix, only
		/// the elements "seen" by this MatrixRef are touched; you can
		/// use this property to fill a rectangular region of a matrix
		/// by combining `cut` and `fill`).
		void fill(ElementT value) {
			fill(value, [](const ElementT&,
			               const ElementT&) { return true; });
		}

		/// This version of `fill` only writes those elements that are
		/// accepted by the given predicate. The predicate is used in
		/// the same way as `copyFrom` (see its docs for details).
		template <typename Pred>
		void fill(ElementT value, Pred predicate) {
			auto nr = rows(), nc = cols();
			for(int i=0; i < nr; i++)
				for(int j=0; j < nc; j++)
					if (predicate((*this)[{i, j}], value))
						(*this)[{i, j}] = value;
		}

		/// Cut a rectangular slice of the matrix corresponding to the
		/// given region. An invalid region will result in a std::range_error.
		Slice cut(Region<> region) {
			if (!(region.offset[0] >= 0 && region.offset[1] >= 0 &&
			      region.size[0] > 0 && region.size[1] > 0 &&
			      region.offset[0] + region.size[0] <= n_rows_ &&
			      region.offset[1] + region.size[1] <= n_cols_))
				throw std::range_error("region is out of bounds");

			unsigned row_stride = n_cols_ - region.size[1];
			unsigned offset = flatIndex(region.offset[0], region.offset[1]);
			// Cast to unsigned is always valid, because we checked
			// for region.size[*] > 0
			return {ptr_,
					(unsigned) region.size[0], (unsigned) region.size[1],
					offset, row_stride};
		}

		/// Just like `cut`, but return a ConstSlice that doesn't
		/// allow to write data. This method can be called on a const
		/// MatrixRef.
		ConstSlice cutConst(Region<> region) const {
			if (!(region.offset[0] >= 0 &&
			      region.offset[1] >= 0 &&
			      region.offset[0] + region.size[0] <= n_rows_ &&
			      region.offset[1] + region.size[1] <= n_cols_))
				throw std::range_error("region is out of bounds");

			unsigned row_stride = n_cols_ - region.size[1];
			unsigned offset = flatIndex(region.offset[0], region.offset[1]);
			// Cast to unsigned is always valid, because we checked
			// for region.size[*] > 0
			return {ptr_,
					(unsigned) region.size[0], (unsigned) region.size[1],
					offset, row_stride};
		}


		/// Create a copy of the Matrix viewed by this MatrixRef.  The
		/// new buffer will be created through `create`. This method,
		/// used in combination with `cut` (or `cutConst`) allows to
		/// copy a region of the matrix to a new external buffer.
		template <typename CopyPtr = ElementPtr,
		          typename CopySlicePtr = SliceElementPtr>
		MatrixRef<CopyPtr, CopySlicePtr> copy() const {
			typedef MatrixRef<CopyPtr, CopySlicePtr> RetT;

			auto ret = MatrixRef<CopyPtr, CopySlicePtr>::create(rows(), cols());
			ret.copyFrom(*this);
			return ret;
		}

		/// Copy the whole `src` matrix to this one, overwriting every
		/// element. `src` must have the exact same size as this
		/// MatrixRef. To copy only regions of the matrix (meaning
		/// source, destination, or both), use this method in
		/// combination with `cut` (`cutConst` is sufficient for the
		/// source).
		template <typename MatT>
		void copyFrom(const MatT& src) {
			copyFrom(src, [](const ElementT&,
			                 const typename MatT::ElementT&) { return true; });
		}

		/// This version of `copyFrom` behaves like the other one
		/// (without a `predicate` parameter), with the same
		/// restrictions, but only copies the elements that have been
		/// accepted by the given predicate. The predicate should be a
		/// callable with the following signature (or equivalent):
		///
		///    bool func(const ElementA& a, const ElementB& b);
		///
		/// where ElementA and ElementB are the types of the elements
		/// of this matrix and the source's, respectively. For each
		/// element, the predicate is called with the current value
		/// and the potential new value as parameters, in this
		/// order. If it returns false, the element is skipped, and
		/// the old value is maintained; otherwise, it is regularly
		/// transferred.
		template <typename MatT, typename Pred>
		void copyFrom(const MatT& src, Pred predicate)
			{
				assert (rows() == src.rows() || cols() == src.cols());

				// Copy has to be done this way, one element at a time,
				// because copying the array directly would disregard
				// offset and stride (this could be rewritten by defining
				// an iterator and using std::copy, but ain't nobody got
				// time for that)
				for(int i=0; i < rows(); i++)
					for(int j=0; j < cols(); j++) {
						auto& dst_val = (*this)[{i, j}];
						const auto& src_val = src[{i, j}];
						if (predicate(const_cast<const ElementT&>(dst_val), src_val))
							dst_val = src_val;
					}
			}

	private:
		inline size_t flatIndex(size_t row, size_t col) const {
			return offset_ + row * (n_cols_+row_stride_) + col;
		}

		int n_rows_, n_cols_;
		size_t offset_, row_stride_;
		PtrT ptr_;
	};

	/// A version of MatrixRef that safely shares the ownership of the
	/// array it points to with all the other Matrix objects that
	/// "look at" the same underlying array (this includes its slices,
	/// or "block references").  When a Matrix is deleted, and no
	/// other Matrix is still "alive" using the same underlying
	/// buffer, then the buffer is deleted with it; otherwise, the
	/// buffer is left untouched. You can safely pass Matrix objects
	/// around to have different parts of the program work together on
	/// the same data buffer (for example, in different region,
	/// overlapping or not).
	template<typename ElemT>
	using Matrix = MatrixRef<boost::shared_array<ElemT>>;


	/// An infinite-sized matrix that can be read from and written to
	/// in finite-sized regions.
	///
	/// The object internally stores "blocks" of finite size to store
	/// the actual data in memory; blocks are created on-the-fly
	/// whenever needed, and they're initialized with the default
	/// value provided during construction.
	template <typename T = double>
	class SparseMatrix {
	public:
		typedef T ElementT;

		SparseMatrix(Index<> blockSize, ElementT defaultValue)
			: block_size_(blockSize)
			, default_value_(defaultValue)
			{}
		SparseMatrix(SparseMatrix&& other)
			: block_size_(other.block_size_)
			, default_value_(other.default_value_)
			, blocks_(std::move(other)) {}

		inline ElementT& operator[](Index<> index) {
			auto block_id = findBlock(index);
			auto& block = getBlock(block_id);
			return block[index - blockPosition(block_id)];
		}

		// A const version of operator[] is not really possible at the
		// moment: every access could trigger the creation of a new
		// block if the index falls outside already allocated blocks.

		/// Read a piece of the sparse matrix into a new shared buffer
		/// (Matrix<>). The read region is the one passed through the
		/// `region` parameter.
		Matrix<ElementT> read(Region<> region)
			{
				auto ret = Matrix<ElementT>::create(region.size[0],
				                                    region.size[1]);
				read(region.offset, ret);
				return ret;
			}

		/// Read a piece of the sparse matrix into `dest`
		/// The region of interest has its top-left corner at `corner`,
		/// and has the same size as `dest`.
		template <typename MatT>
		void read(Index<> corner, MatT dest)
			{
				Region<> region = {corner, dest.size()};
				for(const auto& piece : spliceIntoBlocks(region)) {
					dest.cut({piece.position, piece.block.size()})
						.copyFrom(piece.block);
				}
			}

		void fill(Region<> region, ElementT value) {
			for(auto& piece : spliceIntoBlocks(region))
				piece.block.fill(value);
		}

		template <typename Pred>
		void fill(Region<> region, ElementT value, Pred predicate) {
			for(auto& piece : spliceIntoBlocks(region))
				piece.block.fill(value, predicate);
		}

		template <typename MatRef>
		void write(const MatRef& matrix, Index<> corner) {
			writeIf(matrix, corner,
			        [](const ElementT&,
			           const typename MatRef::ElementT&) { return true; });
		}

		template <typename MatRef, typename Pred>
		void writeIf(const MatRef& matrix, Index<> corner, Pred predicate) {
			Region<> region {corner, matrix.size()};

			for(auto& piece : spliceIntoBlocks(region)) {
				auto src = matrix.cutConst({piece.position, piece.block.size()});
				piece.block.copyFrom(src, predicate);
			}
		}

	private:
		FRIEND_TEST (SparseMatrixTest, blockSplicing);
		FRIEND_TEST (SparseMatrixTest, internalBlockMgmt);

		typedef Matrix<ElementT> BlockT;
		typedef Index<> BlockIdT;

		struct Piece {
			typedef typename Matrix<ElementT>::Slice BlockT;

			Piece(Index<> position, BlockT blk)
				: position(position), block(blk) {}

			Index<> position;
			BlockT block;
		};

		/// Return the coordinates for the top-left corner of the
		/// block identified by the given BlockId.
		inline Index<> blockPosition(BlockIdT blockId) const {
			return blockId * block_size_;
		}

		/// Get the Block of the block that contains the given element
		BlockIdT findBlock(Index<> offset) const {
			Index<> offset_corr = offset;
			for(int i=0; i < 2; i++)
				if (offset_corr[i] < 0)
					offset_corr[i] += 1;

			// These are integer divisions
			BlockIdT ret = { offset_corr[0] / block_size_[0],
			                 offset_corr[1] / block_size_[1] };
			for(int i=0; i < 2; i++)
				if (offset[i] < 0)
					ret[i] -= 1;

			return ret;
		}

		/// Retrieve the block corresponding to the given Id, or
		/// create it if it doesn't exist already.
		BlockT& getBlock(BlockIdT blockIndex) {
			auto iter = blocks_.find(blockIndex);
			if (iter != blocks_.end())
				return iter->second;

			auto newBlock = BlockT::create(block_size_[0], block_size_[1]);
			newBlock.fill(default_value_);
			auto insert_res = blocks_.insert({blockIndex, newBlock});
			return insert_res.first->second;
		}

		/// Take any rectangular region and "splice" it into pieces
		/// such that each piece fits evenly in a single block, and
		/// each block contains at most one such piece.
		std::vector<Piece> spliceIntoBlocks(const Region<>& region) {
			// Find the coordinates of the pieces
			std::vector<int> points[2];
			for(int ax=0; ax < 2; ax++) {
				int cur = region.offset[ax];
				int rem = region.offset[ax] % block_size_[ax];
				if (rem != 0) {
					points[ax].push_back(cur);
					cur -= rem;
					if (rem > 0)
						cur += block_size_[ax];
				}

				assert ((cur%block_size_[ax]) == 0);
				const int last = region.offset[ax] + region.size[ax] - 1;
				while (cur < last) {
					points[ax].push_back(cur);
					cur += block_size_[ax];
				}

				points[ax].push_back(last+1);
			}

			// Cut each piece corresponding to the previously computed
			// coordinates
			std::vector<Piece> ret;
			auto firstBlock = findBlock(region.offset);
			for(int i=0; i < points[0].size()-1; i++) {
				for(int j=0; j < points[1].size()-1; j++) {
					Index<> corner { points[0][i], points[1][j] };
					Index<> src_offset = corner - region.offset;
					auto blockIndex = firstBlock + Index<>{i, j};
					auto block = getBlock(blockIndex);

					Index<> index_in_block = corner - blockPosition(blockIndex);
					Index<> piece_size {
						points[0][i+1] - points[0][i],
						points[1][j+1] - points[1][j],
					};
					Piece piece {
						src_offset, // position in the spliced matrix
						block.cut({ index_in_block, piece_size })
					};
					ret.push_back(piece);
				}
			}

			return ret;
		}

		Index<> block_size_;
		ElementT default_value_;
		std::unordered_map<BlockIdT, BlockT,
		                   boost::hash<BlockIdT>> blocks_;
	};


}
