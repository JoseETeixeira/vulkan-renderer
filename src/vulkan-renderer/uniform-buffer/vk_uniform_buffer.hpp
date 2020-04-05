#pragma once

#include "../buffers/vk_buffer.hpp"


namespace inexor {
namespace vulkan_renderer {

	
	/// @class InexorUniformBuffer.
	struct InexorUniformBuffer : public InexorBuffer
	{
		// 
		VkDescriptorBufferInfo descriptor_buffer_info;
		
		// 
		VkDescriptorSet descriptor_set;

	};


};
};