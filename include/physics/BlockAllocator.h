#ifndef BLOCKALLOCATOR_H
#define BLOCKALLOCATOR_H

#include "common.h"

namespace ale
{

const int32_t chunkSize = 16 * 1024;
const int32_t maxBlockSize = 640;
const int32_t blockSizes = 14;
const int32_t chunkArrayIncrement = 128;

// 특정 size의 블록 리스트가 보관된 공간
struct Chunk
{
	int32_t blockSize;
	Block *blocks;
};

struct Block
{
	Block *next;
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

	Block *m_availableBlocks[blockSizes];

	static int32_t blockSizes[blockSizes];
	static uint8_t blockSizeLookup[maxBlockSize + 1]; 
	static bool blockSizeLookupInitialized;
};
} // namespace ale

#endif