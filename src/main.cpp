// render_crtn
// Keita Yamada
// 2014.09.05

#define _CRT_SECURE_NO_WARNINGS
#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <time.h>
#include <array>

#include <Imath/ImathVec.h>
#include <Imath/ImathLine.h>
#include <Imath/ImathLineAlgo.h>
#include <Imath/ImathColor.h>
#include <Imath/ImathColorAlgo.h>
#include <Imath/ImathRandom.h>
#include <Imath/ImathSphere.h>
#include <Imath/ImathPlane.h>
#include <Imath/ImathFun.h>

typedef std::vector<Imath::C3f> ImageBuffer;

#define LIMIT_DEPTH 5.0

// camera
struct Camera{
	Imath::V3d pos,dir,up;

	Camera(Imath::V3d pos, Imath::V3d dir, Imath::V3d up)
	{
		this->pos = pos;
		this->dir = dir;
		this->up = up;
	}
};

// render settings
struct Setting{
	int reso_w, reso_h, samples, supersamples;

	Setting(int w, int h, int samples, int supersamples)
	{
		this->reso_w = w;
		this->reso_h = h;
		this->samples = samples;
		this->supersamples = supersamples;
	}
};

// screen
struct Screen{
	double width,height,dist;
	Imath::V3d x, y, center;

	Screen(const Camera& camera, const Setting& setting, double scale=30.0, double dist=40.0)
	{
		this->width = scale * setting.reso_w / setting.reso_h;
		this->height = scale;
		this->dist = dist;
		this->x = camera.dir.cross(camera.up).normalized() * this->width;
		this->y = camera.dir.cross(this->x).normalized()   * this->height;
		this->center = camera.pos + camera.dir * this->dist;
	}
};

// ray
struct Ray : Imath::Line3d{
	Ray(const Camera& camera, const Screen& screen, const Setting& setting, int x, int y, int supersample_x, int supersample_y)
	:Imath::Line3d()
	{
		double rate = 1.0 / setting.supersamples;
		double rx = supersample_x * rate + rate / 2.0;
		double ry = supersample_y * rate + rate / 2.0;
		Imath::V3d end = screen.center + screen.x * ((rx + x) / setting.reso_w - 0.5) + screen.y * ((ry + y) / setting.reso_h - 0.5);
		this->pos = camera.pos;
		this->dir = (end - this->pos).normalized();
	}

	Ray(Imath::V3d pos, Imath::V3d dir) :Imath::Line3d()
	{
		this->pos = pos;
		this->dir = dir;
	}
};

// selection of reflection algorithm
enum Reflection
{
	Diffuse, Specular, Refraction
};

// (interface) instance of mesh
struct BaseMesh{

	virtual bool intersect_ray(Ray& ray, Imath::V3d& intersection_point, Imath::V3d& intersection_normal)
	{
		return false;
	}
};

// sphere
struct Sphere : BaseMesh, Imath::Sphere3d{
		
	Sphere(Imath::V3d pos, double radius)
	:BaseMesh(), Imath::Sphere3d(pos,radius)
	{

	}

	virtual bool intersect_ray(Ray& ray, Imath::V3d& intersection_point, Imath::V3d& intersection_normal) override
	{
		if (this->intersect(ray, intersection_point))
		{
			intersection_normal = (intersection_point - this->center).normalized();
			return true;
		}
		else
		{
			return false;
		}
	}
};

// triangle
struct Triangle : Imath::Plane3d{
	Imath::V3d v0, v1, v2;

	Triangle(Imath::V3d v0, Imath::V3d v1, Imath::V3d v2) : Imath::Plane3d(v0, v1, v2)
	{
		this->v0 = v0;
		this->v1 = v1;
		this->v2 = v2;
	}
};

double distance(Imath::V3d& a, Imath::V3d& b)
{
	return sqrt(pow(a.x - b.x, 2) + pow(a.y - b.y, 2) + pow(a.z - b.z, 2));
}

// mesh
struct Mesh : BaseMesh, std::vector<Triangle>{

	Mesh() : BaseMesh()
	{
		this->clear();
	}

	virtual bool intersect_ray(Ray& ray, Imath::V3d& intersection_point, Imath::V3d& intersection_normal) override
	{
		Imath::V3d hit_point, barycentric;
		bool front;
		double dist;
		double current_dist = HUGE_VALD;
		int cnt = 0;
		int id = -1;
		for (auto &t : *(this))
		{
			front = false;
			if (Imath::intersect(ray, t.v0, t.v1, t.v2, hit_point, barycentric, front))
			{
				if (front)
				{
					dist = distance(hit_point, ray.pos);
					if (dist < current_dist)
					{
						intersection_point = hit_point;
						current_dist = dist;
						id = cnt;
					}

				}
			}
			cnt++;
		}

		if (current_dist < HUGE_VALD)
		{
			intersection_normal = this->at(id).normal;
			return true;
		}
		else
		{
			return false;
		}
	}

};


// geometry
struct Geometry{
	BaseMesh& mesh;
	Imath::V3d pos;
	Imath::C3f color, emission;
	Reflection reflection;

	Geometry(BaseMesh& mesh, Imath::V3d pos, Imath::C3f color = Imath::C3f(0, 0, 0), Imath::C3f emission = Imath::C3f(0, 0, 0), Reflection reflection = Reflection::Diffuse)
		:mesh(mesh)
	{
		this->pos = pos;
		this->color = color;
		this->emission = emission;
		this->reflection = reflection;
	}
};

// scene
struct Scene{

	std::vector<Geometry> geometries;
	Imath::C3f bgColor;

	Scene(Imath::C3f bgColor = Imath::C3f(0,0,0))
	{
		geometries.clear();
		this->bgColor = bgColor;
	}

	bool intersect(Ray ray, Imath::V3d& intersection_point, Imath::V3d& intersection_normal, int& obj_id)
	{
		double dist;
		double current_dist = HUGE_VALD;
		Imath::V3d hit_point, hit_normal;
		obj_id = -1;
		int cnt = 0;
		for (auto &g : geometries)
		{
			if (g.mesh.intersect_ray(ray, hit_point, hit_normal))
			{
				dist = distance(hit_point, ray.pos);
				if (dist < current_dist)
				{
					intersection_point = hit_point;
					intersection_normal = hit_normal;
					current_dist = dist;
					obj_id = cnt;
				}
			}
			cnt++;
		}
		return current_dist < HUGE_VALD;
	}
};

Imath::C3f radiance(Scene& scene, Ray& ray, Imath::Rand48& rnd, const int depth)
{
	Imath::V3d intersection_point, intersection_normal;
	int obj_id;
	if (!scene.intersect(ray, intersection_point, intersection_normal, obj_id))
	{
		return scene.bgColor;
	}

	Geometry geom = scene.geometries[obj_id];

	Imath::V3d orienting_normal = intersection_normal;
	if ((intersection_normal ^ ray.dir) > 0.0)
	{
		orienting_normal *= -1.0;
	}

	if (depth > LIMIT_DEPTH)
	{
		return geom.emission;
	}

	float reflect_ratio = (5.0f - static_cast<float>(depth)) / 5.0f;

	Imath::C3f inc_rad(0,0,0), weight(1,1,1);

	switch (geom.reflection)
	{
	case Reflection::Diffuse:
	{
		Imath::V3d w, u, v;
		w = orienting_normal;
		if (fabs(w.x) > 0.0000009)
		{
			u = (Imath::V3d(0.0, 1.0, 0.0) % w).normalized();
		}
		else
		{
			u = (Imath::V3d(1.0, 0.0, 0.0) % w).normalized();
		}
		v = w % u;
		double r1 = 2.0 * M_PI * rnd.nextf();
		double r2 = rnd.nextf();
		double rr2 = sqrt(r2);
		ray.pos = intersection_point;
		ray.dir = (u * cos(r1) * rr2 + v * sin(r1) * rr2 + w * sqrt(1.0 - r2)).normalized();
		inc_rad = radiance(scene, ray, rnd, depth + 1);
		weight = geom.color * reflect_ratio;
	}
		break;

	case Reflection::Specular:
		ray.pos = intersection_point;
		ray.dir -= intersection_normal * 2.0 * (intersection_normal ^ ray.dir);
		inc_rad = radiance(scene, ray, rnd, depth + 1);
		weight = geom.color * reflect_ratio;
		break;

	case Reflection::Refraction:
		bool into = (orienting_normal ^ intersection_normal) > 0.0;

		double default_refraction = 1.0;
		double object_refraction = 1.5;
		double ray_refraction;
		if (into)
		{
			ray_refraction = default_refraction / object_refraction;
		}
		else
		{
			ray_refraction = object_refraction / default_refraction;
		}
		double incident_dot = ray.dir ^ orienting_normal;
		double critical_factor = 1.0 - pow(ray_refraction, 2) * (1.0 - pow(incident_dot,2));

		Ray reflection_ray(intersection_point, ray.dir - intersection_normal * 2.0 * (intersection_normal ^ ray.dir));
		Ray refraction_ray(intersection_point, (ray.dir * ray_refraction - intersection_normal * (into ? 1.0 : -1.0) * (incident_dot * ray_refraction + sqrt(critical_factor))).normalized());

		// total reflection
		if (critical_factor < 0.0)
		{
			inc_rad = radiance(scene, reflection_ray, rnd, depth + 1);
			weight = geom.color * reflect_ratio;
			break;
		}

		double a = object_refraction - default_refraction;
		double b = object_refraction + default_refraction;
		double vertical_incidence_factor = pow(a, 2) / pow(b, 2);
		double c = 1.0 - (into ? -1.0 * incident_dot : (refraction_ray.dir ^ -1.0 * orienting_normal));
		double fresnel_incidence_factor = vertical_incidence_factor + (1.0 - vertical_incidence_factor) * pow(c,5);
		double radiance_scale = pow(ray_refraction, 2.0);
		double refraction_factor = (1.0 - fresnel_incidence_factor) * radiance_scale;

		double probability = 0.75 + fresnel_incidence_factor;
		if (depth > 2)
		{
			if (rnd.nextf() < probability)
			{
				inc_rad = radiance(scene, reflection_ray, rnd, depth + 1) * fresnel_incidence_factor;
				weight = geom.color * reflect_ratio;
			}
			else
			{
				inc_rad = radiance(scene, refraction_ray, rnd, depth + 1) * refraction_factor;
				weight = geom.color * reflect_ratio;
			}
		}
		else
		{
			inc_rad = 
				radiance(scene, reflection_ray, rnd, depth + 1) * fresnel_incidence_factor +
				radiance(scene, refraction_ray, rnd, depth + 1) * refraction_factor;
			weight = geom.color * reflect_ratio;
		}		

		break;
	}

	return geom.emission + weight * inc_rad;

}

typedef struct tagBITMAPFILEHEADER {
	unsigned short bfType;
	unsigned long  bfSize;
	unsigned short bfReserved1;
	unsigned short bfReserved2;
	unsigned long  bfOffBits;
} BITMAPFILEHEADER;

typedef struct tagBITMAPINFOHEADER{
	unsigned long  biSize;
	long           biWidth;
	long           biHeight;
	unsigned short biPlanes;
	unsigned short biBitCount;
	unsigned long  biCompression;
	unsigned long  biSizeImage;
	long           biXPixPerMeter;
	long           biYPixPerMeter;
	unsigned long  biClrUsed;
	unsigned long  biClrImporant;
} BITMAPINFOHEADER;

int f2c(float value)
{
	return static_cast<unsigned char>(Imath::clamp(value, 0.0f, 1.0f) * 255);
}


void write_bmp(const char* imagename, ImageBuffer& buffer, const Setting& setting)
{
	unsigned int scan_line_bytes = setting.reso_w * 3;
	int file_size = sizeof(BITMAPFILEHEADER)+sizeof(BITMAPINFOHEADER)+scan_line_bytes * setting.reso_h;
	BITMAPFILEHEADER header;
	header.bfType = 'B' | ('M' << 8);
	header.bfSize = file_size;
	header.bfReserved1 = 0;
	header.bfReserved2 = 0;
	header.bfOffBits = sizeof(BITMAPFILEHEADER) + sizeof(BITMAPINFOHEADER);
	BITMAPINFOHEADER infoHeader;
	infoHeader.biSize = sizeof(BITMAPINFOHEADER);
	infoHeader.biWidth = setting.reso_w;
	infoHeader.biHeight = setting.reso_h;
	infoHeader.biPlanes = 1;
	infoHeader.biBitCount = 24;
	infoHeader.biCompression = 0;
	infoHeader.biSizeImage = setting.reso_w * setting.reso_h * 3;
	infoHeader.biXPixPerMeter = 3780;
	infoHeader.biYPixPerMeter = 3780;
	infoHeader.biClrUsed = 0;
	infoHeader.biClrImporant = 0;
	
	FILE *fp = fopen(imagename, "wb");
	int i, j, k, index;
	unsigned char buf;

	fwrite(&header.bfType, sizeof(header.bfType), 1, fp);
	fwrite(&header.bfSize, sizeof(header.bfSize), 1, fp);
	fwrite(&header.bfReserved1, sizeof(header.bfReserved1), 1, fp);
	fwrite(&header.bfReserved2, sizeof(header.bfReserved2), 1, fp);
	fwrite(&header.bfOffBits, sizeof(header.bfOffBits), 1, fp);

	fwrite(&infoHeader.biSize, sizeof(infoHeader.biSize), 1, fp);
	fwrite(&infoHeader.biWidth, sizeof(infoHeader.biWidth), 1, fp);
	fwrite(&infoHeader.biHeight, sizeof(infoHeader.biHeight), 1, fp);
	fwrite(&infoHeader.biPlanes, sizeof(infoHeader.biPlanes), 1, fp);
	fwrite(&infoHeader.biBitCount, sizeof(infoHeader.biBitCount), 1, fp);
	fwrite(&infoHeader.biCompression, sizeof(infoHeader.biCompression), 1, fp);
	fwrite(&infoHeader.biSizeImage, sizeof(infoHeader.biSizeImage), 1, fp);
	fwrite(&infoHeader.biXPixPerMeter, sizeof(infoHeader.biXPixPerMeter), 1, fp);
	fwrite(&infoHeader.biYPixPerMeter, sizeof(infoHeader.biYPixPerMeter), 1, fp);
	fwrite(&infoHeader.biClrUsed, sizeof(infoHeader.biClrUsed), 1, fp);
	fwrite(&infoHeader.biClrImporant, sizeof(infoHeader.biClrImporant), 1, fp);

	for (i = 0; i < (int)setting.reso_h; i++)
	{

		for (j = 0; j < (int)setting.reso_w; j++) {
			index = (setting.reso_h - i - 1) * setting.reso_w + j;
			buf = f2c(buffer[index].y);
			fwrite(&buf, 1, 1, fp);
			buf = f2c(buffer[index].x);
			fwrite(&buf, 1, 1, fp);
			buf = f2c(buffer[index].z);
			fwrite(&buf, 1, 1, fp);
		}
	}
	fclose(fp);	
}


void write_ppm(const char* imagename, ImageBuffer& buffer, const Setting& setting)
{
	FILE *f;
	f = fopen(imagename, "wb");
	Imath::PackedColor packedBuf;
	fprintf(f, "P3\n%d %d\n%d\n", setting.reso_w, setting.reso_h, 255);
	for (int i = 0; i < setting.reso_w * setting.reso_h; i++){
		packedBuf = Imath::rgb2packed(buffer[i]);
		//fprintf(f, "%d %d %d ", packedBuf & 0x000000FF, (packedBuf >> 8) & 0x000000FF, (packedBuf >> 16) & 0x000000FF);
		fprintf(f, "%d %d %d ", f2c(buffer[i].x), f2c(buffer[i].y), f2c(buffer[i].z));
	}

	fclose(f);
}

int render(const Setting& setting, Camera& camera, Screen& screen, Scene& scene, ImageBuffer& buffer)
{
	int time_count = 1;
	clock_t start_time, current_time;
	float now_time;
	start_time = clock();
	char fname[255];

	for (int y = 0; y < setting.reso_h; y++)
	{
		Imath::Rand48 rnd;
		std::cout << "Rendering (y = " << y << ") " << (100.0 * y / (setting.reso_h - 1)) << "%" << std::endl;

		for (int x = 0; x < setting.reso_w; x++)
		{
			int index = (setting.reso_h - y - 1) * setting.reso_w + x;
			buffer[index] = Imath::C3f(0,0,0);
			
			for (int sy = 0; sy < setting.supersamples; sy++)
			{
				for (int sx = 0; sx < setting.supersamples; sx++)
				{
					Imath::C3f acm_rad = Imath::C3f(0,0,0);

					for (int s = 0; s < setting.samples; s++)
					{
						Ray ray = Ray(camera,screen,setting,x,y,sx,sy);
						acm_rad += radiance(scene, ray, rnd, 0) / setting.samples / (setting.supersamples * setting.supersamples);
						buffer[index] += acm_rad;

						current_time = clock();
						now_time = static_cast<float>((current_time - start_time) / CLOCKS_PER_SEC);
						if (now_time > time_count * 60.0f)
						{
							std::cout << "width:" << setting.reso_w << std::endl;
							std::cout << "height:" << setting.reso_h << std::endl;
							std::cout << "sample:" << setting.samples << std::endl;
							std::cout << "subpixel:" << setting.supersamples << std::endl;
							std::cout << time_count << "minute(s)" << std::endl;
							std::cout << "image output..." << std::endl;
							sprintf(fname, "out_%02d.bmp", time_count);
							write_bmp(fname, buffer, setting);
							time_count++;
						}
					}
				}
			}

		}
	}

	return 0;
}

int main(int argc, char** argv)
{
	int x = 0, y = 0, sample = 0, subpixel = 0;
	int i = 1;
	while (true)
	{
		if (i >= argc) break;

		if (std::string(argv[i]).compare("-x") == 0)
		{
			i++;
			x = atoi(argv[i]);
			i++;
			continue;
		}

		if (std::string(argv[i]).compare("-y") == 0)
		{
			i++;
			y = atoi(argv[i]);
			i++;
			continue;
		}

		if (std::string(argv[i]).compare("-s") == 0)
		{
			i++;
			sample = atoi(argv[i]);
			i++;
			continue;
		}

		if (std::string(argv[i]).compare("-p") == 0)
		{
			i++;
			subpixel = atoi(argv[i]);
			i++;
			continue;
		}

	}

	if (x == 0 || y == 0 || sample == 0 || subpixel == 0)
	{
		std::cout << "argv error. using default setting." << std::endl;
		x = 640;
		y = 480;
		sample = 4;
		subpixel = 2;
	}

	std::cout << "width:" << x << std::endl;
	std::cout << "height:" << y << std::endl;
	std::cout << "sample:" << sample << std::endl;
	std::cout << "subpixel:" << subpixel << std::endl;

	Setting render_settings(x, y, sample, subpixel);
	Camera cam(Imath::V3d(50.0, 52.0, 220.0), Imath::V3d(0.0, -0.04, -30.0).normalized(), Imath::V3d(0.0, -1.0, 0.0));
	Screen screen(cam, render_settings);
	ImageBuffer image(render_settings.reso_w * render_settings.reso_h);

	//Mesh mesh = Mesh();
	//mesh.push_back(Triangle(Imath::V3d(12.8, 5.0, -10), Imath::V3d(5.0, 20.0, -10), Imath::V3d(20.0, 20.0, -10)));
	//Geometry geom(mesh, Imath::V3d(0, 0, 0), Imath::C3f(0.7, 0.5, 0.5));
	//Geometry geom(mesh, Imath::V3d(0, 0, 0), Imath::C3f(0.7, 0.5, 0.5));
	Scene scene;
	//scene.geometries.push_back(geom);
	scene.geometries.push_back(Geometry(Sphere(Imath::V3d(1e5 + 1, 40.8, 81.6),		1e5),	Imath::V3d(1e5 + 1, 40.8, 81.6),	Imath::C3f(0.75, 0.25, 0.25), Imath::C3f(0, 0, 0), Reflection::Diffuse));
	scene.geometries.push_back(Geometry(Sphere(Imath::V3d(-1e5 + 99, 40.8, 81.6),	1e5),	Imath::V3d(-1e5 + 99, 40.8, 81.6),	Imath::C3f(0.25, 0.25, 0.75), Imath::C3f(0, 0, 0), Reflection::Diffuse));
	scene.geometries.push_back(Geometry(Sphere(Imath::V3d(50, 40.8, 1e5),			1e5),	Imath::V3d(50, 40.8, 1e5),			Imath::C3f(0.75, 0.75, 0.75), Imath::C3f(0, 0, 0), Reflection::Diffuse));
	scene.geometries.push_back(Geometry(Sphere(Imath::V3d(50, 40.8, -1e5 + 250),	1e5),	Imath::V3d(50, 40.8, -1e5 + 250),	Imath::C3f(0.0,  0.0,  0.0), Imath::C3f(0, 0, 0), Reflection::Diffuse));
	scene.geometries.push_back(Geometry(Sphere(Imath::V3d(50, 1e5, 81.6),			1e5),	Imath::V3d(50, 1e5, 81.6),			Imath::C3f(0.75, 0.75, 0.75), Imath::C3f(0, 0, 0), Reflection::Diffuse));
	scene.geometries.push_back(Geometry(Sphere(Imath::V3d(50, -1e5 + 81.6, 81.6),	1e5),	Imath::V3d(50, -1e5 + 81.6, 81.6),	Imath::C3f(0.75, 0.75, 0.75), Imath::C3f(0, 0, 0), Reflection::Diffuse));
	scene.geometries.push_back(Geometry(Sphere(Imath::V3d(65, 20, 20),				20),	Imath::V3d(65, 20, 20),				Imath::C3f(0.25, 0.75, 0.25), Imath::C3f(0, 0, 0), Reflection::Diffuse));
	scene.geometries.push_back(Geometry(Sphere(Imath::V3d(27, 16.5, 47),			16.5),	Imath::V3d(27, 16.5, 47),			Imath::C3f(0.99, 0.99, 0.99), Imath::C3f(0, 0, 0), Reflection::Specular));
	scene.geometries.push_back(Geometry(Sphere(Imath::V3d(77, 16.5, 78),			16.5),	Imath::V3d(77, 16.5, 78),			Imath::C3f(0.99, 0.99, 0.99), Imath::C3f(0, 0, 0), Reflection::Refraction));
	scene.geometries.push_back(Geometry(Sphere(Imath::V3d(50, 90, 81.6),			15.0),	Imath::V3d(50, 90, 81.6),			Imath::C3f(0.0,  0.0,  0.0), Imath::C3f(36, 36, 36), Reflection::Diffuse));
	//scene.geometries.push_back(Geometry(Triangle(Imath::V3d(12.8, 5.0, -10), Imath::V3d(5.0, 20.0, -10), Imath::V3d(20.0, 20.0, -10)), Imath::V3d(65, 20, 20), Imath::C3f(0.75, 0.75, 0.75), Imath::C3f(0, 0, 0), Reflection::Diffuse));

	render(render_settings,cam,screen,scene,image);
	write_bmp("out_complete.bmp", image, render_settings);
	return 0;
}
