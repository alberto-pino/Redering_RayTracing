#include <parsers/ass_parser.h>
#include <parsers/text_parser.h>
#include <world.h>

#include <lambert.h>
#include <standard.h>
#include <blinn.h>
#include <blinnphong.h>

bool ReadAssLambertBlock(TextParser& parser, World* the_world)
{
	string mat_name;
	Spectrum Kd(0.5f);
	string Kd_name;
	float kd = 1.0f;

	Spectrum Ka(0.5f);
	string Ka_name;
	float ka = 1.0f;

	bool success = false;
	string sToken;
	while (!parser.Eof())
	{
		success = parser.GetToken(sToken);
		if (success)
		{
			if (sToken == "NAME")
			{
				parser.GetToken(mat_name);
			}
			else if (sToken == "KD")
			{
				kd = parser.GetNumber();
			}
			else if (sToken == "KD_COLOR")
			{
				float rgb[3];
				if (parser.IsNextNumber())
				{
					parser.GetPoint3(rgb[0], rgb[1], rgb[2]);
					Kd = Spectrum(rgb);
				}
				else
				{
					parser.GetToken(Kd_name);
				}
			}
			else if (sToken == "KA")
			{
				ka = parser.GetNumber();
			}
			else if (sToken == "KA_COLOR")
			{
				float rgb[3];
				if (parser.IsNextNumber())
				{
					parser.GetPoint3(rgb[0], rgb[1], rgb[2]);
					Ka = Spectrum(rgb);
				}
				else
				{
					parser.GetToken(Ka_name);
				}
			}
			else if (sToken == "}")
			{
				break;
			}
			else
			{
				parser.SkipLine();
			}
		}
	}

	Color kd_color;
	Color ka_color;
	if (Kd_name.empty())
	{
		kd_color = Color(Kd);
	}
	else
	{
		kd_color = Color(Kd_name.c_str(), Spectrum(kd));
	}
	if (Ka_name.empty())
	{
		ka_color = Color(Ka);
	}
	else
	{
		ka_color = Color(Ka_name.c_str(), Spectrum(ka));
	}

	Lambert* mat = new Lambert(mat_name.c_str(), kd, kd_color);
	the_world->addMaterial(mat);

	return true;
}

bool ReadAssBlinn(TextParser& parser, World* the_world)
{
	string mat_name;
	Spectrum Kd(0.5f);
	float kd = 1.0f;
	string Kd_name;
	Spectrum Ka(0.5f);
	float ka = 1.0f;
	string Ka_name;
	Spectrum Ks(0.0f);
	float ks = 1.0f;
	string Ks_name;
	float power = 16.0f;

	bool success = false;
	string sToken;
	while (!parser.Eof())
	{
		success = parser.GetToken(sToken);
		if (success)
		{
			if (sToken == "NAME")
			{
				parser.GetToken(mat_name);
			}
			else if (sToken == "KD")
			{
				kd = parser.GetNumber();
			}
			else if (sToken == "KD_COLOR")
			{
				if (parser.IsNextNumber())
				{
					float rgb[3];
					parser.GetPoint3(rgb[0], rgb[1], rgb[2]);
					Kd = Spectrum(rgb);
				}
				else
				{
					parser.GetToken(Kd_name);
				}
			}
			else if (sToken == "Ka")
			{
				ka = parser.GetNumber();
			}
			else if (sToken == "KA_COLOR")
			{
				if (parser.IsNextNumber())
				{
					float rgb[3];
					parser.GetPoint3(rgb[0], rgb[1], rgb[2]);
					Ka = Spectrum(rgb);
				}
				else
				{
					parser.GetToken(Ka_name);
				}
			}
			else if (sToken == "KS")
			{
				ks = parser.GetNumber();
			}
			else if (sToken == "KS_COLOR")
			{
				if (parser.IsNextNumber())
				{
					float rgb[3];
					parser.GetPoint3(rgb[0], rgb[1], rgb[2]);
					Ks = Spectrum(rgb);
				}
				else
				{
					parser.GetToken(Ks_name);
				}
			}
			else if (sToken == "POWER")
			{
				power = parser.GetNumber();
			}
			else if (sToken == "}")
			{
				break;
			}
			else
			{
				parser.SkipLine();
			}
		}
	}

	Color c_kd, c_ka, c_ks;
	if (Kd_name.empty())
		c_kd = Color(Kd);
	else
		c_kd = Color(Kd_name.c_str(), Spectrum(kd));
	if (Ka_name.empty())
		c_ka = Color(Ka);
	else
		c_ka = Color(Ka_name.c_str(), Spectrum(ka));
	if (Ks_name.empty())
		c_ks = Color(Ks);
	else
		c_ks = Color(Ks_name.c_str(), Spectrum(ks));
	
	BlinnPhong* mat = new BlinnPhong(mat_name.c_str(), kd, c_kd, ks, c_ks, power);
	the_world->addMaterial(mat);

	return true;
}

bool ReadAssAiStandardBlock(TextParser& parser, World* the_world)
{
	string mat_name;
	Spectrum Kd(0.5f);
	float kd = 1.0f;
	string Kd_name;
	Spectrum Ka(0.5f);
	float ka = 1.0f;
	string Ka_name;
	Spectrum Ks(0.0f);
	float ks = 1.0f;
	string Ks_name;
	Spectrum Kr(0.0f);
	float kr = 1.0f;
	string Kr_name;
	Spectrum Kt(0.0f);
	float kt = 1.0f;
	string Kt_name;

	float kshi = 20.0f;

	bool success = false;
	string sToken;
	while (!parser.Eof())
	{
		success = parser.GetToken(sToken);
		if (success)
		{
			if (sToken == "NAME")
			{
				parser.GetToken(mat_name);
			}
			else if (sToken == "KD")
			{
				kd = parser.GetNumber();
			}
			else if (sToken == "KD_COLOR")
			{
				if (parser.IsNextNumber())
				{
					float rgb[3];
					parser.GetPoint3(rgb[0], rgb[1], rgb[2]);
					Kd = Spectrum(rgb);
				}
				else
				{
					parser.GetToken(Kd_name);
				}
			}
			else if (sToken == "Ka")
			{
				ka = parser.GetNumber();
			}
			else if (sToken == "KA_COLOR")
			{
				if (parser.IsNextNumber())
				{
					float rgb[3];
					parser.GetPoint3(rgb[0], rgb[1], rgb[2]);
					Ka = Spectrum(rgb);
				}
				else
				{
					parser.GetToken(Ka_name);
				}
			}
			else if (sToken == "KS")
			{
				ks = parser.GetNumber();
			}
			else if (sToken == "KS_COLOR")
			{
				if (parser.IsNextNumber())
				{
					float rgb[3];
					parser.GetPoint3(rgb[0], rgb[1], rgb[2]);
					Ks = Spectrum(rgb);
				}
				else
				{
					parser.GetToken(Ks_name);
				}
			}
			else if (sToken == "KR")
			{
				kr = parser.GetNumber();
			}
			else if (sToken == "KR_COLOR")
			{
				if (parser.IsNextNumber())
				{
					float rgb[3];
					parser.GetPoint3(rgb[0], rgb[1], rgb[2]);
					Kr = Spectrum(rgb);
				}
				else
				{
					parser.GetToken(Kr_name);
				}
			}
			else if (sToken == "KT")
			{
				kt = parser.GetNumber();
			}
			else if (sToken == "KT_COLOR")
			{
				if (parser.IsNextNumber())
				{
					float rgb[3];
					parser.GetPoint3(rgb[0], rgb[1], rgb[2]);
					Kt = Spectrum(rgb);
				}
				else
				{
					parser.GetToken(Kt_name);
				}
			}
			else if (sToken == "KSHI")
			{
				kshi = parser.GetNumber();
			}
			else if (sToken == "}")
			{
				break;
			}
			else
			{
				parser.SkipLine();
			}
		}
	}

	Standard* mat = new Standard(mat_name.c_str());
	if (Kd_name.empty())
		mat->Kd = Color(kd * Kd);
	else
		mat->Kd = Color(Kd_name.c_str(), Spectrum(kd));
	mat->Ka = ka;
	if (Ka_name.empty())
		mat->Ka_color = Color(Ka);
	else
		mat->Ka_color = Color(Ka_name.c_str(), Spectrum(ka));
	if (Ks_name.empty())
		mat->Ks = Color(ks * Ks);
	else
		mat->Ks = Color(Ks_name.c_str(), Spectrum(ks));
	if (Kr_name.empty())
		mat->Kr = Color(kr * Kr);
	else
		mat->Kr = Color(Kr_name.c_str(), Spectrum(kr));
	if (Kt_name.empty())
		mat->Kt = Color(kt * Kt);
	else
		mat->Kt = Color(Kt_name.c_str(), Spectrum(kt));
	mat->Kshi = kshi;
	the_world->addMaterial(mat);

	return true;
}

bool CustomReadAssBlock(const string& sToken, TextParser& parser, World* the_world)
{
	if (sToken == "LAMBERT")
	{
		ReadAssLambertBlock(parser, the_world);
		return true;
	}
	else if (sToken == "AISTANDARD")
	{
		ReadAssAiStandardBlock(parser, the_world);
		return true;
	}
	else if (sToken == "BLINN")
	{
		ReadAssBlinn(parser, the_world);
		return true;
	}
	else
	{
		return false;
	}	
}