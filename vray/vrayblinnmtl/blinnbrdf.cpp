#include "dynamic.h"
#include "vraybase.h"
#include "vrayinterface.h"
#include "vraycore.h"
#include "vrayrenderer.h"

#include "blinnbrdf.h"

#include "shadedata_new.h"
#include "max.h"





extern EVALFUNC NormalEvalFunc;
extern EVALFUNC EvalFunc;

using namespace VUtils;

void MyBaseBSDF::init(const VRayContext &rc, const Color &reflectionColor, real reflectionGlossiness,
                      int subdivs, const Color &transp, int dblSided, const Color &diffuse,
                      wcWeaveParameters *weave_parameters) {
    

	reflect_filter=reflectionColor;
	diffuse_color=diffuse;
	transparency=transp;
	doubleSided=dblSided;
	origBackside=rc.rayresult.realBack;

    m_weave_parameters = weave_parameters;

	// Dim the diffuse color by the reflection
	if (nsamples>0) {
		diffuse_color*=reflect_filter.whiteComplement();
	}

	// Dim the diffuse/reflection colors by the transparency
	Color invTransp=transp.whiteComplement();
	diffuse_color*=invTransp;
	reflect_filter*=invTransp;

	const VR::VRaySequenceData &sdata=rc.vray->getSequenceData();

	// Set the normals to use for lighting
	normal=rc.rayresult.normal;
	gnormal=rc.rayresult.gnormal;
	if (rc.rayresult.realBack) {
		normal=-normal;
		gnormal=-gnormal;
	}
	//HERE! Perturb these normals for bump!

	/* 
		BUMP TEST
	*/

	float perturbed_normal_x, perturbed_normal_y, perturbed_normal_z;

#ifdef DYNAMIC
    if(EvalFunc){
        float cs=(float) (direction*normal);
		if (cs<0.0f) cs=0.0f;
		return cs*NormalEvalFunc(rc,weave_parameters,&perturbed_normal_x,
			&perturbed_normal_y, &perturbed_normal_z);
    } else {
        return Color(1.f,0.f,1.f);
    }
#else
    float cs=(float) (direction*normal);
	if (cs<0.0f) cs=0.0f;
	return cs*dynamic_eval(rc,&perturbed_normal_x,
		&perturbed_normal_y, &perturbed_normal_z);
#endif

	    wcIntersectionData intersection_data;
   
    const VR::VRayInterface &vri_const=static_cast<const VR::VRayInterface&>(rc);
	VR::VRayInterface &vri=const_cast<VR::VRayInterface&>(vri_const);
	ShadeContext &sc=static_cast<ShadeContext&>(vri);

    Point3 uv = sc.UVW(1);

    //Convert wi and wo to the correct coordinate system

    //NOTE(Vidar): These are in world space...
    Point3 p, d;
    p.x = rc.rayparams.viewDir.x;
    p.y = rc.rayparams.viewDir.y;
    p.z = rc.rayparams.viewDir.z;
    p = sc.VectorFrom(p,REF_WORLD);
    p = p.Normalize();

    // UVW derivatives
    Point3 dpdUVW[3];
    sc.DPdUVW(dpdUVW,1);

    //TODO(Vidar): I'm not certain that we want these dot products...
    Point3 n_vec = sc.Normal().Normalize();
    Point3 u_vec = dpdUVW[0].Normalize();
    Point3 v_vec = dpdUVW[1].Normalize();
    u_vec = v_vec ^ n_vec;
    v_vec = n_vec ^ u_vec;

    Matrix3 mat(u_vec, v_vec, n_vec, Point3(0.f,0.f,0.f));
    mat.Invert(); // TODO(Vidar) transposing would be better...

    Point3 localViewDir = (p * mat).Normalize();
    Point3 localDir     = (d * mat).Normalize();

    VUtils::Vector wo;
    wo.x = localViewDir.x;
    wo.y = localViewDir.y;
    wo.z = localViewDir.z;
    VUtils::Vector wi;
    wi.x = localDir.x;
    wi.y = localDir.y;
    wi.z = localDir.z;

    intersection_data.wi_x = wi.x;
    intersection_data.wi_y = wi.y;
    intersection_data.wi_z = wi.z;

    intersection_data.wo_x = wo.x;
    intersection_data.wo_y = wo.y;
    intersection_data.wo_z = wo.z;

    intersection_data.uv_x = uv.x;
    intersection_data.uv_y = uv.y;


    wcPatternData dat = wcGetPatternData(intersection_data,weave_parameters);

	Point3 rcnormal;
	rcnormal.x = rc.rayresult.normal.x;
	rcnormal.y = rc.rayresult.normal.y;
	rcnormal.z = rc.rayresult.normal.z;

	Point3 dP[2];
	sc.BumpBasisVectors(dP,AXIS_UV);
	Point3 dPdu = dP[0];
	Point3 dPdv = dP[1];

	Point3 yarn_normal;
	yarn_normal.x = dat.normal_x;
	yarn_normal.y = dat.normal_y;
	yarn_normal.z = dat.normal_z;

	Point3 new_dPdu = dPdu + rcnormal * (-yarn_normal.x - rcnormal * Point3(dPdu));
	Point3 new_dPdv = dPdv + rcnormal * (-yarn_normal.y - rcnormal * Point3(dPdv));

	Point3 perturbed_normal = new_dPdu ^ new_dPdv;

	normal.x = perturbed_normal.x;
	normal.y = perturbed_normal.y;
	normal.z = perturbed_normal.z;

	/*
	            Float dDispDu = data.normal_x;
            Float dDispDv = data.normal_y;
            Vector dpdv = its.dpdv + its.shFrame.n * (
                    -dDispDv - dot(its.shFrame.n, its.dpdv));
            Vector dpdu = its.dpdu + its.shFrame.n * (
                    -dDispDu - dot(its.shFrame.n, its.dpdu));
					*/


	/*  END BUMP TEST  */






	// Check if we need to trace the reflection
	dontTrace=p_or(
		p_or(rc.rayparams.totalLevel>=sdata.params.options.mtl_maxDepth, 0==sdata.params.options.mtl_reflectionRefraction),
		p_and(0!=(rc.rayparams.rayType & RT_INDIRECT), !sdata.params.gi.reflectCaustics)
	);

	if (dontTrace) {
		reflect_filter.makeZero();
	} else {
		glossiness=remapGlossiness(reflectionGlossiness);
		nsamples=subdivs*subdivs;

		// Compute the normal matrix
		if (rc.rayparams.viewDir*normal>0.0f) computeNormalMatrix(rc, gnormal, nm);
		else computeNormalMatrix(rc, normal, nm);

		inm=inverse(nm);
	}
}

// From BRDFSampler
Color MyBaseBSDF::getDiffuseColor(Color &lightColor) {
	Color res=diffuse_color*lightColor;
	lightColor*=transparency;
	return res;
}

Color MyBaseBSDF::getLightMult(Color &lightColor) {
	Color res=(diffuse_color+reflect_filter)*lightColor;
	lightColor*=transparency;
	return res;
}

Color MyBaseBSDF::eval(const VRayContext &rc, const Vector &direction, Color &lightColor, Color &origLightColor, float probLight, int flags) {
#ifdef DYNAMIC
    if(EvalFunc){
        float cs=(float) (direction*normal);
		if (cs<0.0f) cs=0.0f;
        return cs*EvalFunc(rc,direction,lightColor,origLightColor,probLight,flags,m_weave_parameters,nm);
    } else {
        return Color(1.f,0.f,1.f);
    }
#else
    float cs=(float) (direction*normal);
	if (cs<0.0f) cs=0.0f;
    return cs*dynamic_eval(rc,direction,lightColor,origLightColor,probLight,flags,m_weave_parameters,nm);
#endif
    /*
	// Skip this part if diffuse component is not required
	if (0!=(flags & FBRDF_DIFFUSE)) {
		float cs=(float) (direction*normal);
		if (cs<0.0f) cs=0.0f;
		float probReflection=2.0f*cs;

		float k=getReflectionWeight(probLight, probReflection);
		res+=(0.5f*probReflection*k)*(diffuse_color*lightColor);
	}

	// Skip this part if specular component is not required
	if (!dontTrace && glossiness>=0.0f && 0!=(flags & FBRDF_SPECULAR)) {
		float probReflection=getGlossyProbability(direction, rc.rayparams.viewDir);
		float k=getReflectionWeight(probLight, probReflection);
		res+=(0.5f*probReflection*k)*(reflect_filter*lightColor);
	}

	lightColor*=transparency;
	origLightColor*=transparency;

	return res;*/
}

void MyBaseBSDF::traceForward(VRayContext &rc, int doDiffuse) {
	BRDFSampler::traceForward(rc, doDiffuse);
	if (doDiffuse) rc.mtlresult.color+=rc.evalDiffuse()*diffuse_color;
}

int MyBaseBSDF::getNumSamples(const VRayContext &rc, int doDiffuse) {
	return p_or(rc.rayparams.currentPass==RPASS_GI, (rc.rayparams.rayType & RT_LIGHT)!=0, glossiness<0.0f)? 0 : nsamples;
}

Color MyBaseBSDF::getTransparency(const VRayContext &rc) {
	return transparency;
}

VRayContext* MyBaseBSDF::getNewContext(const VRayContext &rc, int &samplerID, int doDiffuse) {
	if (2==doDiffuse || dontTrace || nsamples==0) return NULL;

	// Create a new context
	VRayContext &nrc=rc.newSpawnContext(2, reflect_filter, RT_REFLECT | RT_GLOSSY | RT_ENVIRONMENT, normal);

	// Set up the new context
	nrc.rayparams.dDdx.makeZero(); // Zero out the directional derivatives
	nrc.rayparams.dDdy.makeZero();
	nrc.rayparams.mint=0.0f; // Set the ray extents
	nrc.rayparams.maxt=1e18f;
	nrc.rayparams.tracedRay.p=rc.rayresult.wpoint; // Set the new ray origin to be the surface hit point
	return &nrc;
}

ValidType MyBaseBSDF::setupContext(const VRayContext &rc, VRayContext &nrc, float uc, int doDiffuse) {
	if (glossiness<0.0f) {
		// Pure reflection
		Vector dir=getReflectDir(rc.rayparams.viewDir, rc.rayresult.normal);

		// If the reflection direction is below the surface, use the geometric normal
		real r0=-(real) (rc.rayparams.viewDir*rc.rayresult.gnormal);
		real r1=(real) (dir*rc.rayresult.gnormal);
		if (r0*r1<0.0f) dir=getReflectDir(rc.rayparams.viewDir, rc.rayresult.gnormal);

		// Set ray derivatives
		VR::getReflectDerivs(rc.rayparams.viewDir, dir, rc.rayparams.dDdx, rc.rayparams.dDdy, nrc.rayparams.dDdx, nrc.rayparams.dDdy);

		// Set the direction into the ray context
		nrc.rayparams.tracedRay.dir=nrc.rayparams.viewDir=dir;
		nrc.rayparams.tracedRay.p=rc.rayresult.wpoint; // Set the new ray origin to be the surface hit point
		nrc.rayparams.mint=0.0f; // Set the ray extents
		nrc.rayparams.maxt=1e18f;
	} else {
		// Compute a Blinn reflection direction
		Vector dir=getGlossyReflectionDir(uc, BRDFSampler::getDMCParam(nrc, 1), rc.rayparams.viewDir, nrc.rayparams.rayProbability);


		// If this is below the surface, ignore
		if (dir*gnormal<0.0f) return false;

		// Set ray derivatives
		VR::getReflectDerivs(rc.rayparams.viewDir, dir, rc.rayparams.dDdx, rc.rayparams.dDdy, nrc.rayparams.dDdx, nrc.rayparams.dDdy);

		// Set the direction into the ray context
		nrc.rayparams.tracedRay.dir=nrc.rayparams.viewDir=dir;
	}

	return true;
}

RenderChannelsInfo* MyBaseBSDF::getRenderChannels(void) { return &RenderChannelsInfo::reflectChannels; }

void MyBaseBSDF::computeNormalMatrix(const VR::VRayContext &rc, const VR::Vector &normal, VR::Matrix &nm) {
		makeNormalMatrix(normal, nm);
}

// From BSDFSampler
BRDFSampler* MyBaseBSDF::getBRDF(BSDFSide side) {
	if (!doubleSided) {
		if (side==bsdfSide_back) return NULL; // There is nothing on the back side
		return static_cast<VR::BRDFSampler*>(this);
	} else {
		if (side==bsdfSide_front) {
			if (origBackside) return NULL;
		} else {
			if (!origBackside) return NULL;
		}
		return static_cast<BRDFSampler*>(this);
	}
}
