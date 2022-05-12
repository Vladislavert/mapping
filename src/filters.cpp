#include "filters.hpp"

double	Filters::median(double *data, unsigned int sizeData)
{
	double			result;
	unsigned int	indexCenter;

	std::sort(data, data + sizeData);
	indexCenter = round(sizeData/2);
	result = data[indexCenter];

	return (result);
}