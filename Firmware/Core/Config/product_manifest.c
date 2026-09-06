#include "Core/Config/product_manifest.h"

#include "product_catalog.h"

const ProductManifest *ProductManifest_Get(void)
{
	const ProductCatalogEntry *entry = ProductCatalog_GetCurrent();
	return entry != 0 ? &entry->manifest : 0;
}
