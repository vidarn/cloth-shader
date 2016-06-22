#include "dynamic.h"
#include "vraybase.h"
#include "vrayinterface.h"
#include "vraycore.h"
#include "vrayrenderer.h"

#include "blinnbrdf.h"

#ifdef DYNAMIC
extern EVALSPECULARFUNC EvalSpecularFunc;
extern EVALDIFFUSEFUNC  EvalDiffuseFunc;
#endif

using namespace VUtils;

void
MyBaseBSDF::init(const VRayContext &rc, wcWeaveParameters *weave_parameters) {
    m_weave_parameters = weave_parameters;

    EvalDiffuseFunc(rc,weave_parameters,&diffuse_color);

	const VR::VRaySequenceData &sdata=rc.vray->getSequenceData();

	// Set the normals to use for lighting
	normal=rc.rayresult.normal;
	gnormal=rc.rayresult.gnormal;
	if (rc.rayresult.realBack) {
		normal=-normal;
		gnormal=-gnormal;
	}

    orig_backside = rc.rayresult.realBack;

    // Compute the normal matrix
    if (rc.rayparams.viewDir*normal>0.0f) computeNormalMatrix(rc, gnormal, nm);
    else computeNormalMatrix(rc, normal, nm);

    inm=inverse(nm);
}

// From BRDFSampler
VUtils::Color MyBaseBSDF::getDiffuseColor(VUtils::Color &lightColor) {
    VUtils::Color ret = diffuse_color*lightColor;
	lightColor = VUtils::Color(0.f,0.f,0.f);
    return ret;
}

VUtils::Color MyBaseBSDF::getLightMult(VUtils::Color &lightColor) {
    float s = m_weave_parameters->specular_strength;
    VUtils::Color ret = (diffuse_color + VUtils::Color(s,s,s)) * lightColor;
	lightColor = VUtils::Color(0.f,0.f,0.f);
    return ret;
}

VUtils::Color MyBaseBSDF::eval(const VRayContext &rc, const Vector &direction, VUtils::Color &lightColor, VUtils::Color &origLightColor, float probLight, int flags) {
    float factor = direction * rc.rayresult.normal;
    factor = factor < 0.0f ? 0.0f : factor;

    VUtils::Color reflect_color;
    EvalSpecularFunc(rc,direction,m_weave_parameters,nm,&reflect_color);

    VUtils::Color ret(
        (diffuse_color.r + reflect_color.r) * lightColor.r * factor,
        (diffuse_color.g + reflect_color.g) * lightColor.g * factor,
        (diffuse_color.b + reflect_color.b) * lightColor.b * factor);

    lightColor     = VUtils::Color(0.f,0.f,0.f);
    origLightColor = VUtils::Color(0.f,0.f,0.f);

    return ret;
}

void MyBaseBSDF::traceForward(VRayContext &rc, int doDiffuse) {
	BRDFSampler::traceForward(rc, doDiffuse);
	if (doDiffuse) rc.mtlresult.color+=rc.evalDiffuse()*diffuse_color;
}

int MyBaseBSDF::getNumSamples(const VRayContext &rc, int doDiffuse) {
    return 0;
}

VUtils::Color MyBaseBSDF::getTransparency(const VRayContext &rc) {
	return VUtils::Color(0.f,0.f,0.f);
}

VRayContext* MyBaseBSDF::getNewContext(const VRayContext &rc, int &samplerID, int doDiffuse) {
    return NULL;
}

ValidType MyBaseBSDF::setupContext(const VRayContext &rc, VRayContext &nrc, float uc, int doDiffuse) {
    return false;
}

RenderChannelsInfo* MyBaseBSDF::getRenderChannels(void) { return &RenderChannelsInfo::reflectChannels; }

void MyBaseBSDF::computeNormalMatrix(const VR::VRayContext &rc, const VR::Vector &normal, VR::Matrix &nm) {
		makeNormalMatrix(normal, nm);
}

// From BSDFSampler
BRDFSampler* MyBaseBSDF::getBRDF(BSDFSide side) {
    if (side==bsdfSide_front) {
        if (orig_backside) return NULL;
    } else {
        if (!orig_backside) return NULL;
    }
    return static_cast<BRDFSampler*>(this);
}
