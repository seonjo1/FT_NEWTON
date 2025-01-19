#ifndef BLOCKALLOCATOR_H
#define BLOCKALLOCATOR_H

#include "common.h"

namespace ale
{

const int32_t CHUNK_SIZE = 16 * 1024;
const int32_t MAX_BLOCK_SIZE = 640;
const int32_t BLOCK_SIZE_COUNT = 14;
const int32_t CHUNK_ARRAY_INCREMENT = 128;

struct Block
{
	Block *next;
};

// 특정 size의 블록 리스트가 보관된 공간
struct Chunk
{
	int32_t blockSize;
	Block *blocks;
};

class BlockAllocator
{
  public:
	BlockAllocator();
	~BlockAllocator();

	void *allocateBlock(int32_t size);
	void freeBlock(void *pointer, int32_t size);

  private:
	Chunk *m_chunks;		// 전체 청크 메모리리
	int32_t m_chunkCount;	// 사용 중인 청크 수
	int32_t m_chunkSpace;	// 남은 청크 공간

	Block *m_availableBlocks[BLOCK_SIZE_COUNT];

	static int32_t blockSizes[BLOCK_SIZE_COUNT];
	static uint8_t blockSizeLookup[MAX_BLOCK_SIZE + 1]; 
	static bool blockSizeLookupInitialized;
};
} // namespace ale

#endif