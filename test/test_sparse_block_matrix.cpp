#include "heightmap/sparse_block_matrix.hpp"
#include <gtest/gtest.h>

#include <string>
#include <iostream>
#include <algorithm>
#include <random>

namespace heightmap {

	std::ostream& operator<<(std::ostream& os, Index<> const& index) {
		int i;
		os << index[0] << "," << index[1];
		return os;
	}

	std::ostream& operator<<(std::ostream& os, Region<> const& region) {
		os << "Region{" << region.offset << " @ " << region.size << "}";
		return os;
	}

	template <typename ElemPtr, typename SlicePtr>
	std::ostream& operator<<(std::ostream& os,
	                         const MatrixRef<ElemPtr, SlicePtr>& matrix)
	{
		Index<> size = matrix.size();
		for(int i=0; i < size[0]; i++) {
			if (i == 0)
				os << "[ ";
			else
				os << "  ";

			for(int j=0; j < size[1]; j++)
				os << matrix[{i, j}] << "\t";

			os << "\n";
		}
		os << "]";
	}

	TEST(IndexTest, IndexBinOpDistribute) {
		Index<int, 2> a {3, 4};
		Index<int, 2> b {9, 11};

		EXPECT_EQ(a + b, (Index<int,2>{ 12, 15 }));
		EXPECT_EQ(a - b, (Index<int,2>{ -6, -7 }));
	}

	TEST(RegionTest, contains) {
		// reg size should at least be {3,3}
		Region<> reg = { {3,2}, {5, 9} };

		EXPECT_TRUE(reg.contains(reg.offset));
		EXPECT_TRUE(reg.contains(reg.offset + reg.size - Index<>{2,2}));
		EXPECT_TRUE(reg.contains(reg.offset + reg.size - Index<>{1,1}));

		EXPECT_FALSE(reg.contains(reg.offset - Index<>{1,1}));
		EXPECT_FALSE(reg.contains(reg.offset - Index<>{1,0}));
		EXPECT_FALSE(reg.contains(reg.offset - Index<>{0,1}));
		EXPECT_FALSE(reg.contains(reg.offset + reg.size));
		EXPECT_FALSE(reg.contains(reg.offset + reg.size - Index<>{0,1}));
		EXPECT_FALSE(reg.contains(reg.offset + reg.size - Index<>{1,0}));
	}

	struct MatrixRefTest : public ::testing::Test {
		static const unsigned N = 10, M = 12;

		boost::shared_array<float> data;
		Matrix<float> matrix;

		virtual void SetUp() {
			float *ptr = new float[N*M];
			for(int i=0; i < N*M; i++)
				ptr[i] = (float)i * 2;

			data = boost::shared_array<float>(ptr);
			matrix = Matrix<float>(data, N, M);
		}
	};

	TEST_F(MatrixRefTest, bySubscript)  {
		float *ptr = data.get();
		for(int i=0; i < N; i++)
			for(int j=0; j < M; j++)
				EXPECT_EQ( (matrix[{i, j}]), ptr[i * M + j] );
	}

	TEST_F(MatrixRefTest, cutInvalidRegion) {
		Region<> invalid_regions[] = {
			{ {-2,  0},  { 5,   5} }, // offset x < 0
			{ { 0, -2},  { 5,   5} }, // offset y < 0
			{ { 2,  2},  {30,   5} }, // too many rows
			{ {-2,  0},  { 5,  30} }, // too many rows
		};

		for(const Region<>& region : invalid_regions) {
			EXPECT_THROW( matrix.cut(region), std::range_error );
		}
	}

	template<typename PtrA, typename PtrB>
	bool operator==(const MatrixRef<PtrA>& a,
	                const MatrixRef<PtrB>& b)
	{
		unsigned nr = a.rows(), nc = a.cols();

		if (nr != b.rows() || nc != b.cols())
			return false;

		for(int i=0; i < nr; i++) {
			for(int j=0; j < nc; j++) {
				Index<> idx {i, j};
				if (a[idx] != b[idx])
					return false;
			}
		}

		return true;
	}

	TEST_F(MatrixRefTest, cutValidRegion) {
		Region<> ok_regions[] = {
			{ {2, 5}, {3, 4} },
			{ {0, 0}, {matrix.rows(), matrix.cols()} },
		};

		for(const Region<>& ok_region : ok_regions) {
			Matrix<float>::Slice submtx = matrix.cut(ok_region);

			ASSERT_EQ (submtx.rows(), ok_region.size[0]);
			ASSERT_EQ (submtx.cols(), ok_region.size[1]);

			for(int i=0; i < ok_region.size[0]; i++) {
				for(int j=0; j < ok_region.size[1]; j++) {
					Index<> idx {i, j};
					auto a = submtx[idx];
					auto top_idx = ok_region.offset + idx;
					auto b = matrix[top_idx];

					EXPECT_EQ (a, b);
				}
			}

		}
	}

	TEST_F(MatrixRefTest, copySanityCheck) {
		auto mtx_copy = matrix.copy();

		ASSERT_EQ (mtx_copy.rows(), matrix.rows());
		ASSERT_EQ (mtx_copy.cols(), matrix.cols());

		unsigned nr = matrix.rows(), nc = matrix.cols();
		for(int i=0; i < nr; i++) {
			for(int j=0; j < nc; j++) {
				Index<> idx {i, j};
				EXPECT_EQ (matrix[idx], mtx_copy[idx]);
			}
		}
	}

	TEST_F(MatrixRefTest, regionCopySanityCheck) {
		Region<> region = { {2,5}, {3,4} };
		auto reg_copy = matrix.cut(region).copy();

		ASSERT_EQ (reg_copy.rows(), region.size[0]);
		ASSERT_EQ (reg_copy.cols(), region.size[1]);

		unsigned nr = region.size[0], nc = region.size[1];
		for(int i=0; i < nr; i++) {
			for(int j=0; j < nc; j++) {
				Index<> idx {i, j};
				EXPECT_EQ (matrix[region.offset+idx], reg_copy[idx]);
			}
		}
	}

	TEST_F(MatrixRefTest, writeIf) {
		std::mt19937 gen;
		std::uniform_real_distribution<> dis(0, 10);

		static const size_t nr = 25, nc = 25;
		auto matrix = Matrix<float>::create(25, 25);

		for (int i=0; i < nr; i++)
			for (int j=0; j < nc; j++)
				matrix[{i, j}] = dis(gen);

		auto src = Matrix<float>::create(25, 25);
		src.fill(5);
		auto dst = matrix.copy();
		dst.copyFrom(src, [](float d, float s) { return (s > d); });

		for (int i=0; i < nr; i++) {
			for (int j=0; j < nc; j++) {
				Index<> idx {i, j};
				if (src[idx] > matrix[idx])
					ASSERT_EQ (dst[idx], src[idx]);
				else
					ASSERT_EQ (dst[idx], matrix[idx]);
			}
		}
	}

	TEST_F(MatrixRefTest, fillIf) {
		std::mt19937 gen;
		std::uniform_real_distribution<> dis(0, 10);

		static const size_t nr = 25, nc = 25;
		auto matrix = Matrix<float>::create(25, 25);

		for (int i=0; i < nr; i++)
			for (int j=0; j < nc; j++)
				matrix[{i, j}] = dis(gen);

		auto dst = matrix.copy();
		dst.fill(5.0, [](float d, float s) { return (s > d); });

		for (int i=0; i < nr; i++) {
			for (int j=0; j < nc; j++) {
				Index<> idx {i, j};
				if (matrix[idx] < 5.0)
					ASSERT_EQ (dst[idx], 5.0);
				else
					ASSERT_EQ (dst[idx], matrix[idx]);
			}
		}
	}

	struct SparseMatrixTest : public ::testing::Test {};

	TEST_F(SparseMatrixTest, readWriteSanity) {
		SparseMatrix<float> sparse_matrix {{8, 8}, 0};

		auto matrix = Matrix<float>::create(10, 13);
		for(int i=0; i < matrix.rows(); i++)
			for(int j=0; j < matrix.cols(); j++)
				matrix[{i, j}] = i*j*j - j;

		sparse_matrix.write(matrix, {-3, -5});
		auto read_matrix = sparse_matrix.read({{-3, -5}, matrix.size()});
		ASSERT_EQ(read_matrix, matrix);
	}

	TEST_F(SparseMatrixTest, blockSplicing) {
		SparseMatrix<float> sm{{3, 4}, 0};

		auto pieces = sm.spliceIntoBlocks({ {-4, -5}, {6, 12} });
		const Region<> expected_blocks[] = {
			{ {0, 0}, {1, 1} },
			{ {1, 0}, {3, 1} },
			{ {4, 0}, {2, 1} },
			{ {0, 1}, {1, 4} },
			{ {1, 1}, {3, 4} },
			{ {4, 1}, {2, 4} },
			{ {0, 5}, {1, 4} },
			{ {1, 5}, {3, 4} },
			{ {4, 5}, {2, 4} },
			{ {0, 9}, {1, 3} },
			{ {1, 9}, {3, 3} },
			{ {4, 9}, {2, 3} },
		};
		const size_t num_exp_blocks =
			sizeof(expected_blocks) / sizeof(expected_blocks[0]);
		const auto iter_first = expected_blocks;
		const auto iter_end = expected_blocks + num_exp_blocks;

		EXPECT_EQ(pieces.size(), num_exp_blocks);
		for(const auto& piece : pieces) {
			auto iter =
				std::find_if(iter_first, iter_end,
				             [&](const Region<>& reg) {
					             return (piece.position == reg.offset &&
					                     piece.block.size() == reg.size);
				             });
			EXPECT_FALSE (iter == iter_end);
		}
	}

	TEST_F(SparseMatrixTest, internalBlockMgmt) {
		SparseMatrix<float> sm {{4, 5}, 0};

		EXPECT_EQ( (Index<>{-1, -1}), (sm.findBlock({-1, -1})) );
		EXPECT_EQ( (Index<>{-1,  0}), (sm.findBlock({-1,  1})) );
		EXPECT_EQ( (Index<>{ 0, -1}), (sm.findBlock({ 1, -1})) );
		EXPECT_EQ( (Index<>{-1, -1}), (sm.findBlock({-4, -5})) );
		EXPECT_EQ( (Index<>{-1,  1}), (sm.findBlock({-4,  5})) );
		EXPECT_EQ( (Index<>{ 1, -1}), (sm.findBlock({ 4, -5})) );
		EXPECT_EQ( (Index<>{-2, -2}), (sm.findBlock({-6, -7})) );

		EXPECT_EQ( (Index<>{0, 0}), (sm.findBlock({3, 2})) );
		EXPECT_EQ( (Index<>{1, 1}), (sm.findBlock({7, 8})) );
		EXPECT_EQ( (Index<>{0, 1}), (sm.findBlock({3, 8})) );
		EXPECT_EQ( (Index<>{1, 0}), (sm.findBlock({7, 2})) );

		EXPECT_EQ( (Index<>{-4, 15}), (sm.blockPosition({-1, 3})) );
	}

	TEST_F(SparseMatrixTest, fill) {
		SparseMatrix<float> sm = { {4,4}, 0 };

		Region<> region = {{2,2}, {6, 8}};
		sm.fill(region, 99);

		// Must have offset on {0,0}
		Region<> larger_region = { {0,0}, {12, 12} };
		auto read_back = sm.read(larger_region);

		for(int i=0; i < larger_region.size[0]; i++) {
			for(int j=0; j < larger_region.size[1]; j++) {
				Index<> idx = {i,j};
				auto value = read_back[idx];

				if (region.contains(idx))
					ASSERT_EQ(99, value);
				else
					ASSERT_EQ(0, value);
			}
		}
	}
}
