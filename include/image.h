#ifndef IMAGE_H
# define IMAGE_H

# include "common.h"
# include "deviceManager.h"

class Image
{
public:
	static VkImageView createSwapChainImageView(VkDevice device, VkImage image, VkFormat format, VkImageAspectFlags aspectFlags, uint32_t mipLevels);
	Image() = default;
	virtual ~Image() = default;
	virtual void clear();
	VkImageView getImageView();

protected:
	void createImage(DeviceManager* deviceManager, uint32_t width, uint32_t height, uint32_t mipLevels, VkSampleCountFlagBits numSamples, VkImageTiling tiling, VkImageUsageFlags usage, VkMemoryPropertyFlags properties);
	void createImageView(VkImageAspectFlags aspectFlags, uint32_t mipLevels);
	uint32_t findMemoryType(VkPhysicalDevice physicalDevice, uint32_t typeFilter, VkMemoryPropertyFlags properties);

	VkDevice device;
	VkImage image;
	VkDeviceMemory imageMemory;
	VkImageView imageView;
	VkFormat imageFormat;
};


class ColorImage : public Image
{
public:
	static std::unique_ptr<ColorImage> create(DeviceManager* deviceManager, VkFormat format, VkExtent2D swapChainExtent);
	ColorImage() = default;
	virtual ~ColorImage() = default;
private:
	void init(DeviceManager* deviceManager, VkFormat format, VkExtent2D swapChainExtent);	
};


class DepthImage : public Image
{
public:
	static std::unique_ptr<DepthImage> create(DeviceManager* deviceManager, VkFormat format, VkExtent2D swapChainExtent);
	DepthImage() = default;
	virtual ~DepthImage() = default;
private:
	void init(DeviceManager* deviceManager, VkFormat format, VkExtent2D swapChainExtent);
};


class TextureImage : public Image
{
public:
	static std::unique_ptr<TextureImage> create(const char* path, DeviceManager* deviceManager, VkCommandPool commandPool);
	virtual ~TextureImage() = default;
	virtual void clear() override;
	VkSampler getSampler();

private:
	void init(const char* path, DeviceManager* deviceManager, VkCommandPool commandPool);
	TextureImage() = default;
	void createTextureImage(const char* path, DeviceManager* deviceManager, VkCommandPool commandPool);
	void createTextureImageView();
	void createTextureSampler(VkPhysicalDevice physicalDevice);
	void generateMipmaps(DeviceManager* deviceManager, VkCommandPool commandPool, int32_t texWidth, int32_t texHeight, uint32_t mipLevels);
	void copyBufferToImage(VkQueue graphicsQueue, VkCommandPool commandPool, VkBuffer buffer, uint32_t width, uint32_t height);
	void transitionImageLayout(VkQueue graphicsQueue, VkCommandPool commandPool, VkImageLayout oldLayout, VkImageLayout newLayout, uint32_t mipLevels);
	VkCommandBuffer beginSingleTimeCommands(VkCommandPool commandPool);
	void endSingleTimeCommands(VkQueue graphicsQueue, VkCommandPool commandPool, VkCommandBuffer commandBuffer);

	VkSampler sampler;
	uint32_t mipLevels;
};

#endif