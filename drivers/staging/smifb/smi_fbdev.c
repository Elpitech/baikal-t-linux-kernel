/*
 * Copyright 2016 SiliconMotion Inc.
 *
 * This file is subject to the terms and conditions of the GNU General
 * Public License version 2. See the file COPYING in the main
 * directory of this archive for more details.
 *
 */

#include <linux/module.h>
#include <drm/drmP.h>
#include <drm/drm_fb_helper.h>
#include <drm/drm_crtc_helper.h>
#include <linux/fb.h>
#include "smi_drv.h"

#ifdef CONFIG_SMIFB_USE_DMA
extern ssize_t dw_fb_write(struct fb_info *info, const char __user *buf,
			size_t count, loff_t *ppos);
extern int dw_fb_ioctl(struct fb_info *info, u_int cmd, u_long arg);
#define SHADOWFB_OFF	0x10000000	/* 256MB - above VRAM size */
#define SHADOW_PGOFF	(SHADOWFB_OFF >> PAGE_SHIFT)
#endif

#ifdef CONFIG_SMIFB_USE_DMA
static int smifb_alloc_shadow_pages(struct smi_framebuffer *gfb, unsigned npages)
{
	int order, i;
	struct page *page;
	unsigned allocated;

	order = get_order(npages << PAGE_SHIFT);
	page = alloc_pages(GFP_HIGHUSER, order);
	if (!page) {
		pr_err("smifb_alloc_shadow_pages: no memory!\n");
		return -ENOMEM;
	}
	allocated = 1 << order;
	gfb->shadow_npages = allocated;
	gfb->shadow_start = page_to_phys(page);
	gfb->shadow_start_page = page++;
	/* page_count for page[0] is OK */
	for (i = 1; i < allocated; i++) {
		init_page_count(page++);
	}

	return 0;
}

static void smifb_free_shadow_pages(struct smi_framebuffer *gfb)
{
	struct page *page;
	int i;

	page = gfb->shadow_start_page;
	page++;
	for (i = 1; i < gfb->shadow_npages; i++) {
		put_page_testzero(page);
		page++;
	}
	__free_pages(gfb->shadow_start_page, get_order(gfb->shadow_npages << PAGE_SHIFT));
	gfb->shadow_start_page = NULL;
	gfb->shadow_start = 0;
	gfb->shadow_npages = 0;
}
#endif

static int smifb_mmap(struct fb_info *info,
			struct vm_area_struct *vma)
{
	struct smi_fbdev *afbdev = info->par;
	struct drm_gem_object *obj;
	struct smi_bo *bo;
	int ret;

#ifdef CONFIG_SMIFB_USE_DMA
	if (vma->vm_pgoff >= SHADOW_PGOFF) {
		phys_addr_t shadow_start = afbdev->gfb.shadow_start;
		struct smi_framebuffer *gfb = &afbdev->gfb;
		unsigned int npages;
		vma->vm_pgoff -= SHADOW_PGOFF;
		vma->vm_page_prot = PAGE_USERIO;
		npages = vma->vm_pgoff + vma_pages(vma);
		if (!shadow_start) {
			ret = smifb_alloc_shadow_pages(gfb, npages);
			if (ret)
				return ret;
			shadow_start = gfb->shadow_start;
		} else if (npages > gfb->shadow_npages) {
			return -EINVAL;
		}
		return remap_pfn_range(vma, vma->vm_start, 
				       page_to_pfn(gfb->shadow_start_page) +
				       vma->vm_pgoff,
				       vma->vm_end - vma->vm_start,
				       vma->vm_page_prot);
	}
#endif
	obj = afbdev->gfb.obj;
	bo = gem_to_smi_bo(obj);	

	return ttm_fbdev_mmap(vma, &bo->bo);
}


static struct fb_ops smifb_ops = {
	.owner = THIS_MODULE,
	.fb_check_var = drm_fb_helper_check_var,
	.fb_set_par = drm_fb_helper_set_par,
#if LINUX_VERSION_CODE < KERNEL_VERSION(4,3,0)
	.fb_fillrect = sys_fillrect,
	.fb_copyarea = sys_copyarea,
	.fb_imageblit = sys_imageblit,
#else
	.fb_fillrect = drm_fb_helper_cfb_fillrect,
	.fb_copyarea = drm_fb_helper_cfb_copyarea,
	.fb_imageblit = drm_fb_helper_cfb_imageblit,
#endif
	.fb_pan_display = drm_fb_helper_pan_display,
	.fb_blank = drm_fb_helper_blank,
	.fb_setcmap = drm_fb_helper_setcmap,
	.fb_mmap = smifb_mmap,
#ifdef CONFIG_SMIFB_USE_DMA
	.fb_write = dw_fb_write,
	.fb_ioctl = dw_fb_ioctl,
#endif
};

static int smifb_create_object(struct smi_fbdev *afbdev,
			       const struct drm_mode_fb_cmd2 *mode_cmd,
			       struct drm_gem_object **gobj_p)
{
	struct drm_device *dev = afbdev->helper.dev;
	u32 size;
	struct drm_gem_object *gobj;

	int ret = 0;
	size = mode_cmd->pitches[0] * mode_cmd->height;
	ret = smi_gem_create(dev, size, true, &gobj);
	if (ret)
		return ret;

	*gobj_p = gobj;
	return ret;
}



void
smi_fb_zfill(struct drm_device *dev, struct smi_fbdev *gfbdev)
{
	struct fb_info *info = gfbdev->helper.fbdev;
	struct fb_fillrect rect;

	/* Clear the entire fbcon.  The drm will program every connector
	 * with it's preferred mode.  If the sizes differ, one display will
	 * quite likely have garbage around the console.
	 */
	rect.dx = rect.dy = 0;
	rect.width = info->var.xres_virtual;
	rect.height = info->var.yres_virtual;
	rect.color = 0;
	rect.rop = ROP_COPY;
	info->fbops->fb_fillrect(info, &rect);
}



static int smifb_create(struct drm_fb_helper *helper,
			   struct drm_fb_helper_surface_size *sizes)
{
#if LINUX_VERSION_CODE > KERNEL_VERSION(3,14,0)
	struct smi_fbdev *gfbdev =
		container_of(helper, struct smi_fbdev, helper);
#else
	struct smi_fbdev *gfbdev = (struct smi_fbdev *)helper;
#endif		

	struct drm_device *dev = gfbdev->helper.dev;
	struct smi_device *cdev = gfbdev->helper.dev->dev_private;
	struct fb_info *info;
	struct drm_framebuffer *fb;
	struct drm_mode_fb_cmd2 mode_cmd;
#if LINUX_VERSION_CODE < KERNEL_VERSION(4,3,0)
	struct device *device = &dev->pdev->dev;
#endif
	struct drm_gem_object *gobj = NULL;
	struct smi_bo *bo = NULL;
	int size, ret;

	mode_cmd.width = sizes->surface_width;
	mode_cmd.height = sizes->surface_height;	
	mode_cmd.pitches[0] = ((mode_cmd.width) * (sizes->surface_bpp) / 8 + 15) & ~15;
	mode_cmd.pixel_format = drm_mode_legacy_fb_format(sizes->surface_bpp,
							  sizes->surface_depth);
	size = mode_cmd.pitches[0] * mode_cmd.height;

	ret = smifb_create_object(gfbdev, &mode_cmd, &gobj);
	if (ret) {
		DRM_ERROR("failed to create fbcon backing object %d\n", ret);
		return ret;
	}

	bo = gem_to_smi_bo(gobj);
	ret = smi_bo_reserve(bo, false);
	if (ret)
		return ret;

	ret = ttm_bo_kmap(&bo->bo, 0, bo->bo.num_pages,
			  &bo->kmap);
	if (ret) {
		DRM_ERROR("failed to kmap fbcon\n");
		smi_bo_unreserve(bo);
		return ret;
	}

	ttm_bo_unreserve(&bo->bo);


#if LINUX_VERSION_CODE < KERNEL_VERSION(4,3,0)
	info = framebuffer_alloc(0, device);
	if (info == NULL)
		return -ENOMEM;
#else
	info = drm_fb_helper_alloc_fbi(helper);
	if (IS_ERR(info))
		return PTR_ERR(info);
#endif
	info->par = gfbdev;

	ret = smi_framebuffer_init(cdev->dev, &gfbdev->gfb, &mode_cmd, gobj);
	if (ret)
		return ret;

	gfbdev->size = size;

	fb = &gfbdev->gfb.base;
	if (!fb) {
		DRM_INFO("fb is NULL\n");
		return -EINVAL;
	}

	/* setup helper */
	gfbdev->helper.fb = fb;
	/* save pointer to fb_info */
	to_smi_framebuffer(fb)->fbdev = gfbdev->helper.fbdev;
#if LINUX_VERSION_CODE < KERNEL_VERSION(4,3,0)	
	gfbdev->helper.fbdev = info;
#endif

	memset_io(bo->kmap.virtual, 0x0, size);

	strcpy(info->fix.id, "smidrmfb");

	info->flags = FBINFO_DEFAULT;
	info->fbops = &smifb_ops;

#if KERNEL_VERSION(4, 11, 0) > LINUX_VERSION_CODE
	drm_fb_helper_fill_fix(info, fb->pitches[0], fb->depth);
#else
	drm_fb_helper_fill_fix(info, fb->pitches[0], fb->format->depth);
#endif
	
	drm_fb_helper_fill_var(info, &gfbdev->helper, sizes->fb_width,
			       sizes->fb_height);

#if LINUX_VERSION_CODE < KERNEL_VERSION(4,3,0)
	/* setup aperture base/size for vesafb takeover */
	info->apertures = alloc_apertures(1);
	if (!info->apertures) {
		ret = -ENOMEM;
		goto out_iounmap;
	}
#endif
	info->apertures->ranges[0].base = cdev->dev->mode_config.fb_base;
	info->apertures->ranges[0].size = cdev->mc.vram_size;

	info->fix.smem_start = cdev->dev->mode_config.fb_base;
	info->fix.smem_len = cdev->mc.vram_size;

	info->screen_base = bo->kmap.virtual; 
	info->screen_size = size;

	drm_vma_offset_remove(&bo->bo.bdev->vma_manager, &bo->bo.vma_node);
	
	info->fix.mmio_start = 0;
	info->fix.mmio_len = size;
	
#if LINUX_VERSION_CODE < KERNEL_VERSION(4,3,0)
	ret = fb_alloc_cmap(&info->cmap, 256, 0);
	if (ret) {
		DRM_ERROR("%s: can't allocate color map\n", info->fix.id);
		ret = -ENOMEM;
		goto out_iounmap;
	}
#endif

	smi_fb_zfill(dev, gfbdev);

	DRM_INFO("fb mappable at 0x%lX\n", info->fix.smem_start);
	DRM_INFO("vram aper at 0x%lX\n", (unsigned long)info->fix.smem_start);
	DRM_INFO("size %lu\n", (unsigned long)info->fix.smem_len);
#if KERNEL_VERSION(4, 11, 0) > LINUX_VERSION_CODE
	DRM_INFO("fb depth is %d\n", fb->depth);
#else
	DRM_INFO("fb depth is %d\n", fb->format->depth);
#endif
	DRM_INFO("   pitch is %d\n", fb->pitches[0]);

	return 0;
	
#if LINUX_VERSION_CODE < KERNEL_VERSION(4,3,0)
out_iounmap:
	return ret;
#endif	
}

void
smi_fb_output_poll_changed(struct smi_device *sdev)
{

	if (sdev->mode_info.gfbdev)
		drm_fb_helper_hotplug_event(&sdev->mode_info.gfbdev->helper);
	
}



static int smi_fbdev_destroy(struct drm_device *dev,
				struct smi_fbdev *gfbdev)
{
#if LINUX_VERSION_CODE < KERNEL_VERSION(4,3,0)
	struct fb_info *info;
	struct smi_framebuffer *gfb = &gfbdev->gfb;

	if (gfbdev->helper.fbdev) {
		info = gfbdev->helper.fbdev;

		unregister_framebuffer(info);
		if (info->cmap.len)
			fb_dealloc_cmap(&info->cmap);
		framebuffer_release(info);
	}
#else
	struct smi_framebuffer *gfb = &gfbdev->gfb;

	drm_fb_helper_unregister_fbi(&gfbdev->helper);
	drm_fb_helper_release_fbi(&gfbdev->helper);

#endif
	if (gfb->obj) {
		drm_gem_object_unreference_unlocked(gfb->obj);
		gfb->obj = NULL;
	}

#ifdef CONFIG_SMIFB_USE_DMA
	if (gfb->has_dma)
		smi_stop_dma(gfb);
	if (gfb->shadow_start)
		smifb_free_shadow_pages(gfb);
#endif

	drm_fb_helper_fini(&gfbdev->helper);
	drm_framebuffer_unregister_private(&gfb->base);
	drm_framebuffer_cleanup(&gfb->base);

	return 0;
}

static const struct drm_fb_helper_funcs smi_fb_helper_funcs = {
	.gamma_set = smi_crtc_fb_gamma_set,
	.gamma_get = smi_crtc_fb_gamma_get,
	.fb_probe = smifb_create,
};

int smi_fbdev_init(struct smi_device *cdev)
{
	
	int ret = 0;
	


	struct smi_fbdev *gfbdev;
	int bpp_sel = smi_bpp;
	
	gfbdev = kzalloc(sizeof(struct smi_fbdev), GFP_KERNEL);
	if (!gfbdev)
		return -ENOMEM;

	cdev->mode_info.gfbdev = gfbdev;
#if KERNEL_VERSION(3, 17, 0) > LINUX_VERSION_CODE
	gfbdev->helper.funcs = &smi_fb_helper_funcs;
#endif	
	spin_lock_init(&gfbdev->dirty_lock);
#if KERNEL_VERSION(3, 17, 0) <= LINUX_VERSION_CODE
	drm_fb_helper_prepare(cdev->dev, &gfbdev->helper,
								  &smi_fb_helper_funcs);
#endif

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,11,0)
		ret = drm_fb_helper_init(cdev->dev, &gfbdev->helper,
									SMIFB_CONN_LIMIT);
#else
		ret = drm_fb_helper_init(cdev->dev, &gfbdev->helper,
					 cdev->num_crtc, SMIFB_CONN_LIMIT);
#endif

	if (ret)
		goto free;

	ret = drm_fb_helper_single_add_all_connectors(&gfbdev->helper);
	if(ret)
		goto fini;
	/* disable all the possible outputs/crtcs before entering KMS mode */
	drm_helper_disable_unused_functions(cdev->dev);
	ret = drm_fb_helper_initial_config(&gfbdev->helper, bpp_sel);
	if(ret)
		goto fini;

	return 0;

fini:
	drm_fb_helper_fini(&gfbdev->helper);
free:
	kfree(gfbdev);
	cdev->mode_info.gfbdev = NULL;
	
	return ret;


}

void smi_fbdev_fini(struct smi_device *cdev)
{
	if (!cdev->mode_info.gfbdev)
		return;

	smi_fbdev_destroy(cdev->dev, cdev->mode_info.gfbdev);
	kfree(cdev->mode_info.gfbdev);
	cdev->mode_info.gfbdev = NULL;
}
