#include "physics/StackAllocator.h"

namespace ale
{
StackAllocator::StackAllocator() : m_index(0), m_allocation(0), m_entryCount(0) {};

StackAllocator::~StackAllocator()
{
	while (m_entryCount >= 0)
	{
		StackEntry* entry = m_entries + m_entryCount;
		
		if (entry->usedMalloc)
		{
			free(entry->data);
		}
		
		--m_entryCount;
	}
}

void *StackAllocator::allocateStack(int32_t size)
{
	if (m_entryCount == MAX_STACK_ENTRY_SIZE)
	{
		return nullptr;
	}

	StackEntry *entry = m_entries + m_entryCount;
	entry->size = size;
	if (m_index + size > STACK_SIZE)
	{
		entry->data = (char *)malloc(size);
		entry->usedMalloc = true;
	}
	else
	{
		entry->data = m_data + m_index;
		entry->usedMalloc = false;
		m_index += size;
	}

	m_allocation += size;
	++m_entryCount;

	return entry->data;
}

void StackAllocator::freeStack()
{
	if (m_entryCount == 0)
	{
		return;
	}

	StackEntry *entry = m_entries + m_entryCount - 1;

	if (entry->usedMalloc)
	{
		std::free(entry->data);
	}
	else
	{
		m_index -= entry->size;
	}

	m_allocation -= entry->size;
	--m_entryCount;
}

} // namespace ale