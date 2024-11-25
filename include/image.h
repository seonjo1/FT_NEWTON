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
	void clear(VkDevice device);
	VkImageView getImageView();

protected:
	virtual void init(DeviceManager* deviceManager, VkFormat format, VkExtent2D swapChainExtent) = 0;
	void createImage(DeviceManager* deviceManager, uint32_t width, uint32_t height, uint32_t mipLevels, VkSampleCountFlagBits numSamples, VkImageTiling tiling, VkImageUsageFlags usage, VkMemoryPropertyFlags properties);
	uint32_t findMemoryType(VkPhysicalDevice physicalDevice, uint32_t typeFilter, VkMemoryPropertyFlags properties);
	void createImageView(VkDevice device, VkImageAspectFlags aspectFlags, uint32_t mipLevels);

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
	virtual void init(DeviceManager* deviceManager, VkFormat format, VkExtent2D swapChainExtent) override;	
};

class DepthImage : public Image
{
public:
	static std::unique_ptr<DepthImage> create(DeviceManager* deviceManager, VkFormat format, VkExtent2D swapChainExtent);
	DepthImage() = default;
	virtual ~DepthImage() = default;
private:
	virtual void init(DeviceManager* deviceManager, VkFormat format, VkExtent2D swapChainExtent) override;
};

// class Texture : public Image
// {
// public:
// 	static std::unique_ptr<Texture> create();
// 	virtual ~Texture() = default;

// private:
// 	Texture() = default;
//     VkSampler textureSampler;
// 	uint32_t mipLevels;
// };

#endif