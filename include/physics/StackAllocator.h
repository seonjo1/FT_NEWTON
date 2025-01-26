#ifndef STACKALLOCATOR_H
#define STACKALLOCATOR_H

#include "common.h"

namespace ale
{

const int32_t STACK_SIZE = 10 * 1024 * 1024; // 100k
const int32_t MAX_STACK_ENTRY_SIZE = 32;

struct StackEntry
{
	char *data;
	bool usedMalloc;
	int32_t size;
};

class StackAllocator
{
  public:
	StackAllocator();
	~StackAllocator();

	void *allocateStack(int32_t size);
	void freeStack();

  private:
	char m_data[STACK_SIZE];
	int32_t m_index;

	int32_t m_allocation;

	StackEntry m_entries[MAX_STACK_ENTRY_SIZE];
	int32_t m_entryCount;
};
} // namespace ale

#endif