#pragma once

// These will improve the readability of the connection definition

#define getT(Idx) template get<Idx>()
#define connectT(Idx, target) template connect<Idx>(target)
#define getParameterT(Idx) template getParameter<Idx>()
#define setParameterT(Idx, value) template setParameter<Idx>(value)
#define setParameterWT(Idx, value) template setWrapParameter<Idx>(value)
using namespace scriptnode;
using namespace snex;
using namespace snex::Types;

namespace higain_impl
{
// ==============================| Node & Parameter type declarations |==============================

template <int NV>
using svf1_t = wrap::data<filters::svf<NV>, 
                          data::external::filter<0>>;
template <int NV>
using svf7_t = wrap::data<filters::svf<NV>, 
                          data::external::filter<1>>;
template <int NV>
using svf8_t = wrap::data<filters::svf<NV>, 
                          data::external::filter<2>>;
template <int NV> struct sampleandramp
{
	SNEX_NODE(sampleandramp);
	// A compile time array of ramper objects
	// (sfloat is a built-in SNEX type for
	// ramping between values for parameter
	// smoothing.
	span<sfloat, 2> rampers;
	double sampleRate = 44100.0;
	double fadeTimeMs = 1.0 / 44100.0 * 1000.0;
	// Initialise the processing specs here
	void prepare(PrepareSpecs ps)
	{
		sampleRate = ps.sampleRate;
		// set the fade time again
		// (for some reason, iterating over the arra
		// throws a SNEX compilation error, so we have
		// to spell it out for each element)...
		rampers[0].prepare(sampleRate, fadeTimeMs);
		rampers[1].prepare(sampleRate, fadeTimeMs);
	}
	// Reset the processing pipeline here
	void reset()
	{
		// This will reset the ramp position, so when you bypass / unbypass the node it will
		// not pick up from the old value and cause clicks.
		rampers[0].reset();
		rampers[1].reset();
	}
	float calculateSample(float input, sfloat& ramper)
	{
		// if the ramper isn't smoothing, it's waiting for
		// the next input
		if(!ramper.isActive())
			ramper.set(input);
		// this will calculate the next ramp value and
		// update the internal state
		return ramper.advance();
	}
	// Process the signal as frame here
	template <int C> void processFrame(span<float, C>& data)
	{
		data[0] = calculateSample(data[0], rampers[0]);
		if(C != 1)
			data[1] = calculateSample(data[1], rampers[1]);
	}
	// Process the signal here
	template <typename ProcessDataType> void process(ProcessDataType& data)
	{
		// We'll only use frame processing here...
		auto pd = data.toFrameData();
		while(pd.next())
			processFrame(pd.toSpan());
	}
	void handleHiseEvent(HiseEvent& e){}
	void setExternalData(const ExternalData& d, int index){}
	template <int P> void setParameter(double v)
	{
		if(P == 0) // Factor (1 - 64 with step size 1)
		{
			const auto factor = Math.range(v, 0.0, 256.0);
			// internally it will calculate the steps in
			// samples so this is two unnecessary divisions
			// but we get to use the built in type
			fadeTimeMs = (factor / sampleRate) * 1000.0;
			rampers[0].prepare(sampleRate, fadeTimeMs);
			rampers[1].prepare(sampleRate, fadeTimeMs);
		}
	}
};

DECLARE_PARAMETER_RANGE_SKEW(pma1_mod_0Range, 
                             20., 
                             20000., 
                             0.229905);

template <int NV>
using pma1_mod_0 = parameter::from0To1<svf1_t<NV>, 
                                       0, 
                                       pma1_mod_0Range>;

template <int NV>
using pma1_mod_1 = parameter::from0To1<svf7_t<NV>, 
                                       0, 
                                       pma1_mod_0Range>;

template <int NV>
using pma1_mod_2 = parameter::from0To1<svf8_t<NV>, 
                                       0, 
                                       pma1_mod_0Range>;

DECLARE_PARAMETER_RANGE_SKEW(pma1_mod_3Range, 
                             0., 
                             256., 
                             0.5);

template <int NV>
using pma1_mod_3 = parameter::from0To1<sampleandramp<NV>, 
                                       0, 
                                       pma1_mod_3Range>;

DECLARE_PARAMETER_RANGE_STEP(pma1_mod_4Range, 
                             -12., 
                             12., 
                             0.1);

template <int NV>
using pma1_mod_4 = parameter::from0To1<project::Wavefolder<NV>, 
                                       0, 
                                       pma1_mod_4Range>;

template <int NV>
using pma1_mod_5 = parameter::from0To1<filters::allpass<NV>, 
                                       0, 
                                       pma1_mod_0Range>;

template <int NV>
using pma1_mod = parameter::chain<ranges::Identity, 
                                  pma1_mod_0<NV>, 
                                  pma1_mod_1<NV>, 
                                  pma1_mod_2<NV>, 
                                  pma1_mod_3<NV>, 
                                  pma1_mod_4<NV>, 
                                  pma1_mod_5<NV>>;

template <int NV>
using pma1_t = control::pma<NV, pma1_mod<NV>>;

template <int NV>
using chain2_t = container::chain<parameter::empty, 
                                  wrap::fix<1, pma1_t<NV>>>;

template <int NV>
using svf_t = wrap::data<filters::svf<NV>, 
                         data::external::filter<3>>;
template <int NV>
using svf4_t = wrap::data<filters::svf<NV>, 
                          data::external::filter<4>>;
template <int NV>
using svf3_t = wrap::data<filters::svf<NV>, 
                          data::external::filter<5>>;
template <int NV>
using pma_mod_0 = parameter::from0To1<svf_t<NV>, 
                                      0, 
                                      pma1_mod_0Range>;

template <int NV>
using pma_mod_1 = parameter::from0To1<svf4_t<NV>, 
                                      0, 
                                      pma1_mod_0Range>;

template <int NV>
using pma_mod_2 = parameter::from0To1<svf3_t<NV>, 
                                      0, 
                                      pma1_mod_0Range>;

template <int NV> using pma_mod_3 = pma1_mod_3<NV>;

template <int NV> using pma_mod_4 = pma1_mod_5<NV>;

template <int NV> using pma_mod_5 = pma1_mod_4<NV>;

template <int NV>
using pma_mod = parameter::chain<ranges::Identity, 
                                 pma_mod_0<NV>, 
                                 pma_mod_1<NV>, 
                                 pma_mod_2<NV>, 
                                 pma_mod_3<NV>, 
                                 pma_mod_4<NV>, 
                                 pma_mod_5<NV>>;

template <int NV>
using pma_t = control::pma<NV, pma_mod<NV>>;

template <int NV>
using chain1_t = container::chain<parameter::empty, 
                                  wrap::fix<1, pma_t<NV>>>;

template <int NV>
using split_t = container::split<parameter::empty, 
                                 wrap::fix<1, chain2_t<NV>>, 
                                 chain1_t<NV>>;

template <int NV>
using modchain_t_ = container::chain<parameter::empty, 
                                     wrap::fix<1, split_t<NV>>>;

template <int NV>
using modchain_t = wrap::control_rate<modchain_t_<NV>>;

DECLARE_PARAMETER_RANGE_SKEW(xfader_c0Range, 
                             -100., 
                             0., 
                             5.42227);

template <int NV>
using xfader_c0 = parameter::from0To1<core::gain<NV>, 
                                      0, 
                                      xfader_c0Range>;

template <int NV> using xfader_c1 = xfader_c0<NV>;

template <int NV>
using xfader_multimod = parameter::list<xfader_c0<NV>, xfader_c1<NV>>;

template <int NV>
using xfader_t = control::xfader<xfader_multimod<NV>, faders::overlap>;

template <int NV>
using chain8_t = container::chain<parameter::empty, 
                                  wrap::fix<2, core::gain<NV>>>;
namespace custom
{

struct expr
{
	static float op(float input, float value)
	{
		return (1.0f - value) * input + value * Math.sin(Math.PI * 2.0 * value * input);
		;
	}
};
}

template <int NV> using xfader2_c0 = xfader_c0<NV>;

template <int NV> using xfader2_c1 = xfader_c0<NV>;

template <int NV>
using xfader2_multimod = parameter::list<xfader2_c0<NV>, xfader2_c1<NV>>;

template <int NV>
using xfader2_t = control::xfader<xfader2_multimod<NV>, faders::linear>;

template <int NV> using chain11_t = chain8_t<NV>;

template <int NV>
using chain12_t = container::chain<parameter::empty, 
                                   wrap::fix<2, fx::reverb>, 
                                   core::gain<NV>>;

template <int NV>
using split1_t = container::split<parameter::empty, 
                                  wrap::fix<2, chain11_t<NV>>, 
                                  chain12_t<NV>>;

template <int NV>
using chain14_t = container::chain<parameter::empty, 
                                   wrap::fix<2, math::expr<NV, custom::expr>>, 
                                   xfader2_t<NV>, 
                                   split1_t<NV>>;

template <int NV>
using chain20_t = container::chain<parameter::empty, 
                                   wrap::fix<2, svf1_t<NV>>>;

template <int NV>
using chain21_t = container::chain<parameter::empty, 
                                   wrap::fix<2, svf7_t<NV>>>;

template <int NV>
using chain22_t = container::chain<parameter::empty, 
                                   wrap::fix<2, svf8_t<NV>>>;

template <int NV>
using chain30_t = container::chain<parameter::empty, 
                                   wrap::fix<2, filters::allpass<NV>>>;
using stereo_cable = cable::block<2>;

template <int NV>
using chain23_t = container::chain<parameter::empty, 
                                   wrap::fix<2, routing::receive<stereo_cable>>, 
                                   sampleandramp<NV>, 
                                   routing::send<stereo_cable>>;

template <int NV>
using chain35_t = container::chain<parameter::empty, 
                                   wrap::fix<2, routing::receive<stereo_cable>>, 
                                   project::Wavefolder<NV>, 
                                   routing::send<stereo_cable>>;
template <int NV>
using branch4_t = container::branch<parameter::empty, 
                                    wrap::fix<2, chain20_t<NV>>, 
                                    chain21_t<NV>, 
                                    chain22_t<NV>, 
                                    chain30_t<NV>, 
                                    chain23_t<NV>, 
                                    chain35_t<NV>>;

template <int NV> using xfader5_c0 = xfader_c0<NV>;

template <int NV> using xfader5_c1 = xfader_c0<NV>;

template <int NV>
using xfader5_multimod = parameter::list<xfader5_c0<NV>, xfader5_c1<NV>>;

template <int NV>
using xfader5_t = control::xfader<xfader5_multimod<NV>, faders::linear>;

template <int NV> using chain31_t = chain8_t<NV>;

template <int NV>
using limiter_mod = parameter::chain<ranges::Identity, 
                                     parameter::plain<pma_t<NV>, 0>, 
                                     parameter::plain<pma1_t<NV>, 0>>;

template <int NV>
using limiter_t = wrap::mod<limiter_mod<NV>, 
                            wrap::data<dynamics::limiter, data::external::displaybuffer<0>>>;

template <int NV>
using chain37_t = container::chain<parameter::empty, 
                                   wrap::fix<2, limiter_t<NV>>, 
                                   core::gain<NV>>;

template <int NV>
using split7_t = container::split<parameter::empty, 
                                  wrap::fix<2, chain31_t<NV>>, 
                                  chain37_t<NV>>;

template <int NV>
using chain10_t = container::chain<parameter::empty, 
                                   wrap::fix<2, xfader5_t<NV>>, 
                                   split7_t<NV>>;

template <int NV>
using converter_t = control::converter<parameter::plain<fx::sampleandhold<NV>, 0>, 
                                       conversion_logic::ms2samples>;
template <int NV>
using tempo_sync_t = wrap::mod<parameter::plain<converter_t<NV>, 0>, 
                               control::tempo_sync<NV>>;
template <int NV>
using oscillator_t = wrap::no_data<core::oscillator<NV>>;

using chain3_t = container::chain<parameter::empty, 
                                  wrap::fix<2, core::empty>>;

template <int NV>
using chain4_t = container::chain<parameter::empty, 
                                  wrap::fix<2, math::clear<NV>>>;
template <int NV>
using branch_t = container::branch<parameter::empty, 
                                   wrap::fix<2, chain3_t>, 
                                   chain4_t<NV>>;
template <int NV>
using peak_t = wrap::mod<parameter::plain<branch_t<NV>, 0>, 
                         wrap::no_data<core::peak>>;

template <int NV>
using chain13_t = container::chain<parameter::empty, 
                                   wrap::fix<2, tempo_sync_t<NV>>, 
                                   converter_t<NV>, 
                                   math::clear<NV>, 
                                   oscillator_t<NV>, 
                                   fx::sampleandhold<NV>, 
                                   wrap::no_process<math::mod_inv<NV>>, 
                                   math::rect<NV>, 
                                   peak_t<NV>, 
                                   math::clear<NV>>;

template <int NV>
using split4_t = container::split<parameter::empty, 
                                  wrap::fix<2, chain10_t<NV>>, 
                                  chain13_t<NV>>;

template <int NV> using xfader4_c0 = xfader_c0<NV>;

template <int NV> using xfader4_c1 = xfader_c0<NV>;

template <int NV>
using xfader4_multimod = parameter::list<xfader4_c0<NV>, xfader4_c1<NV>>;

template <int NV>
using xfader4_t = control::xfader<xfader4_multimod<NV>, faders::linear>;

template <int NV> using chain27_t = chain8_t<NV>;
using convolution2_t = wrap::data<filters::convolution, 
                                  data::external::audiofile<1>>;

template <int NV>
using chain28_t = container::chain<parameter::empty, 
                                   wrap::fix<2, convolution2_t>, 
                                   core::gain<NV>>;

template <int NV>
using split6_t = container::split<parameter::empty, 
                                  wrap::fix<2, chain27_t<NV>>, 
                                  chain28_t<NV>>;

template <int NV>
using chain26_t = container::chain<parameter::empty, 
                                   wrap::fix<2, xfader4_t<NV>>, 
                                   split6_t<NV>>;

template <int NV> using xfader1_c0 = xfader_c0<NV>;

template <int NV> using xfader1_c1 = xfader_c0<NV>;

template <int NV>
using xfader1_multimod = parameter::list<xfader1_c0<NV>, xfader1_c1<NV>>;

template <int NV>
using xfader1_t = control::xfader<xfader1_multimod<NV>, faders::overlap>;

template <int NV> using chain5_t = chain8_t<NV>;

template <int NV>
using chain_t = container::chain<parameter::empty, 
                                 wrap::fix<2, branch_t<NV>>, 
                                 core::gain<NV>>;

template <int NV>
using chain6_t = container::chain<parameter::empty, 
                                  wrap::fix<2, chain_t<NV>>>;

template <int NV>
using split3_t = container::split<parameter::empty, 
                                  wrap::fix<2, chain5_t<NV>>, 
                                  chain6_t<NV>>;

template <int NV>
using chain16_t = container::chain<parameter::empty, 
                                   wrap::fix<2, svf_t<NV>>>;

template <int NV>
using chain18_t = container::chain<parameter::empty, 
                                   wrap::fix<2, svf4_t<NV>>>;

template <int NV>
using chain17_t = container::chain<parameter::empty, 
                                   wrap::fix<2, svf3_t<NV>>>;

template <int NV> using chain29_t = chain30_t<NV>;

template <int NV> using chain19_t = chain23_t<NV>;

template <int NV> using chain36_t = chain35_t<NV>;
template <int NV>
using branch3_t = container::branch<parameter::empty, 
                                    wrap::fix<2, chain16_t<NV>>, 
                                    chain18_t<NV>, 
                                    chain17_t<NV>, 
                                    chain29_t<NV>, 
                                    chain19_t<NV>, 
                                    chain36_t<NV>>;

template <int NV> using xfader6_c0 = xfader_c0<NV>;

template <int NV> using xfader6_c1 = xfader_c0<NV>;

template <int NV>
using xfader6_multimod = parameter::list<xfader6_c0<NV>, xfader6_c1<NV>>;

template <int NV>
using xfader6_t = control::xfader<xfader6_multimod<NV>, faders::linear>;

template <int NV> using chain33_t = chain8_t<NV>;
using convolution3_t = wrap::data<filters::convolution, 
                                  data::external::audiofile<2>>;

template <int NV>
using chain34_t = container::chain<parameter::empty, 
                                   wrap::fix<2, convolution3_t>, 
                                   core::gain<NV>>;

template <int NV>
using split8_t = container::split<parameter::empty, 
                                  wrap::fix<2, chain33_t<NV>>, 
                                  chain34_t<NV>>;

template <int NV>
using chain32_t = container::chain<parameter::empty, 
                                   wrap::fix<2, xfader6_t<NV>>, 
                                   split8_t<NV>>;

template <int NV>
using chain9_t = container::chain<parameter::empty, 
                                  wrap::fix<2, chain14_t<NV>>, 
                                  branch4_t<NV>, 
                                  core::gain<NV>, 
                                  wrap::no_process<fx::haas<NV>>, 
                                  split4_t<NV>, 
                                  chain26_t<NV>, 
                                  xfader1_t<NV>, 
                                  split3_t<NV>, 
                                  branch3_t<NV>, 
                                  chain32_t<NV>, 
                                  core::gain<NV>, 
                                  core::gain<NV>>;

template <int NV>
using split2_t = container::split<parameter::empty, 
                                  wrap::fix<2, chain8_t<NV>>, 
                                  chain9_t<NV>>;

template <int NV>
using chain7_t = container::chain<parameter::empty, 
                                  wrap::fix<2, xfader_t<NV>>, 
                                  split2_t<NV>>;
template <int NV>
using oversample_t_ = container::chain<parameter::empty, 
                                       wrap::fix<2, chain7_t<NV>>>;

template <int NV>
using oversample_t = wrap::oversample<0, oversample_t_<NV>>;

namespace higain_t_parameters
{
// Parameter list for higain_impl::higain_t --------------------------------------------------------

DECLARE_PARAMETER_RANGE_SKEW(RatioRange, 
                             1, 
                             32., 
                             0.296803);

template <int NV>
using Ratio = parameter::from0To1<higain_impl::limiter_t<NV>, 
                                  3, 
                                  RatioRange>;

DECLARE_PARAMETER_RANGE_STEP(InputFilter_InputRange, 
                             1., 
                             6., 
                             1.);
DECLARE_PARAMETER_RANGE_STEP(InputFilter_0Range, 
                             0., 
                             5., 
                             1.);

template <int NV>
using InputFilter_0 = parameter::from0To1<higain_impl::branch4_t<NV>, 
                                          0, 
                                          InputFilter_0Range>;

template <int NV>
using InputFilter = parameter::chain<InputFilter_InputRange, InputFilter_0<NV>>;

DECLARE_PARAMETER_RANGE_SKEW(InputFilterq_InputRange, 
                             0., 
                             1., 
                             0.264718);
DECLARE_PARAMETER_RANGE_SKEW(InputFilterq_0Range, 
                             0.3, 
                             9.9, 
                             0.264718);

template <int NV>
using InputFilterq_0 = parameter::from0To1<higain_impl::svf1_t<NV>, 
                                           1, 
                                           InputFilterq_0Range>;

DECLARE_PARAMETER_RANGE(InputFilterq_1Range, 
                        0., 
                        0.9);

using InputFilterq_1 = parameter::from0To1<routing::receive<stereo_cable>, 
                                           0, 
                                           InputFilterq_1Range>;

template <int NV>
using InputFilterq_2 = parameter::from0To1<higain_impl::svf8_t<NV>, 
                                           1, 
                                           InputFilterq_0Range>;

template <int NV>
using InputFilterq_3 = parameter::from0To1<higain_impl::svf7_t<NV>, 
                                           1, 
                                           InputFilterq_0Range>;

template <int NV>
using InputFilterq_4 = parameter::from0To1<filters::allpass<NV>, 
                                           1, 
                                           InputFilterq_0Range>;

template <int NV>
using InputFilterq = parameter::chain<InputFilterq_InputRange, 
                                      InputFilterq_0<NV>, 
                                      InputFilterq_1, 
                                      InputFilterq_2<NV>, 
                                      InputFilterq_3<NV>, 
                                      InputFilterq_4<NV>, 
                                      parameter::plain<routing::receive<stereo_cable>, 0>>;

DECLARE_PARAMETER_RANGE_STEP(OutPutMode_InputRange, 
                             1., 
                             6., 
                             1.);
template <int NV>
using OutPutMode_0 = parameter::from0To1<higain_impl::branch3_t<NV>, 
                                         0, 
                                         InputFilter_0Range>;

template <int NV>
using OutPutMode = parameter::chain<OutPutMode_InputRange, OutPutMode_0<NV>>;

template <int NV>
using OutputQ_0 = parameter::from0To1<higain_impl::svf_t<NV>, 
                                      1, 
                                      InputFilterq_0Range>;

template <int NV>
using OutputQ_1 = parameter::from0To1<higain_impl::svf4_t<NV>, 
                                      1, 
                                      InputFilterq_0Range>;

template <int NV>
using OutputQ_2 = parameter::from0To1<higain_impl::svf3_t<NV>, 
                                      1, 
                                      InputFilterq_0Range>;

using OutputQ_3 = InputFilterq_1;

template <int NV> using OutputQ_4 = InputFilterq_4<NV>;

using OutputQ_5 = InputFilterq_1;

template <int NV>
using OutputQ = parameter::chain<ranges::Identity, 
                                 OutputQ_0<NV>, 
                                 OutputQ_1<NV>, 
                                 OutputQ_2<NV>, 
                                 OutputQ_3, 
                                 OutputQ_4<NV>, 
                                 OutputQ_5>;

DECLARE_PARAMETER_RANGE_STEP(scramblerate_InputRange, 
                             0., 
                             18., 
                             1.);
DECLARE_PARAMETER_RANGE_STEP(scramblerate_0Range, 
                             0., 
                             18., 
                             1.);

template <int NV>
using scramblerate_0 = parameter::from0To1<higain_impl::tempo_sync_t<NV>, 
                                           0, 
                                           scramblerate_0Range>;

DECLARE_PARAMETER_RANGE_STEP(scramblerate_1Range, 
                             50., 
                             10000., 
                             1.);

template <int NV>
using scramblerate_1 = parameter::from0To1<project::Wavefolder<NV>, 
                                           1, 
                                           scramblerate_1Range>;

DECLARE_PARAMETER_RANGE_STEP(scramblerate_2Range, 
                             1., 
                             10000., 
                             1.);

template <int NV>
using scramblerate_2 = parameter::from0To1<project::Wavefolder<NV>, 
                                           2, 
                                           scramblerate_2Range>;

template <int NV> using scramblerate_3 = scramblerate_1<NV>;

template <int NV> using scramblerate_4 = scramblerate_2<NV>;

template <int NV>
using scramblerate = parameter::chain<scramblerate_InputRange, 
                                      scramblerate_0<NV>, 
                                      scramblerate_1<NV>, 
                                      scramblerate_2<NV>, 
                                      scramblerate_3<NV>, 
                                      scramblerate_4<NV>>;

DECLARE_PARAMETER_RANGE(Sat_InputRange, 
                        0., 
                        0.25);

template <int NV>
using Sat = parameter::chain<Sat_InputRange, 
                             parameter::plain<math::expr<NV, custom::expr>, 0>>;

DECLARE_PARAMETER_RANGE_STEP(over_InputRange, 
                             1., 
                             5., 
                             1.);
DECLARE_PARAMETER_RANGE_STEP(over_0Range, 
                             0., 
                             4., 
                             1.);

template <int NV>
using over_0 = parameter::from0To1<higain_impl::oversample_t<NV>, 
                                   0, 
                                   over_0Range>;

template <int NV>
using over = parameter::chain<over_InputRange, over_0<NV>>;

DECLARE_PARAMETER_RANGE_SKEW(smooth1_0Range, 
                             0., 
                             1., 
                             0.30103);

template <int NV>
using smooth1_0 = parameter::from0To1<higain_impl::svf1_t<NV>, 
                                      3, 
                                      smooth1_0Range>;

template <int NV>
using smooth1_1 = parameter::from0To1<higain_impl::svf7_t<NV>, 
                                      3, 
                                      smooth1_0Range>;

template <int NV>
using smooth1_2 = parameter::from0To1<higain_impl::svf8_t<NV>, 
                                      3, 
                                      smooth1_0Range>;

template <int NV>
using smooth1 = parameter::chain<ranges::Identity, 
                                 smooth1_0<NV>, 
                                 smooth1_1<NV>, 
                                 smooth1_2<NV>>;

template <int NV>
using smooth2_0 = parameter::from0To1<higain_impl::svf_t<NV>, 
                                      3, 
                                      smooth1_0Range>;

template <int NV>
using smooth2_1 = parameter::from0To1<higain_impl::svf4_t<NV>, 
                                      3, 
                                      smooth1_0Range>;

template <int NV>
using smooth2_2 = parameter::from0To1<higain_impl::svf3_t<NV>, 
                                      3, 
                                      smooth1_0Range>;

template <int NV>
using smooth2 = parameter::chain<ranges::Identity, 
                                 smooth2_0<NV>, 
                                 smooth2_1<NV>, 
                                 smooth2_2<NV>>;

template <int NV>
using Thresh = parameter::plain<higain_impl::limiter_t<NV>, 
                                0>;
template <int NV>
using Attk = parameter::plain<higain_impl::limiter_t<NV>, 
                              1>;
template <int NV>
using Rel = parameter::plain<higain_impl::limiter_t<NV>, 
                             2>;
template <int NV>
using Input = parameter::plain<core::gain<NV>, 0>;
template <int NV> using Out = Input<NV>;
template <int NV>
using Scramble = parameter::plain<higain_impl::xfader1_t<NV>, 
                                  0>;
template <int NV>
using InputFilterCut = parameter::plain<higain_impl::pma1_t<NV>, 
                                        2>;
template <int NV>
using OutputCut = parameter::plain<higain_impl::pma_t<NV>, 2>;
template <int NV>
using InMod = parameter::plain<higain_impl::pma1_t<NV>, 
                               1>;
template <int NV>
using OutMod = parameter::plain<higain_impl::pma_t<NV>, 1>;
template <int NV>
using verb = parameter::plain<higain_impl::xfader2_t<NV>, 
                              0>;
template <int NV>
using mix = parameter::plain<higain_impl::xfader_t<NV>, 
                             0>;
template <int NV>
using Space2 = parameter::plain<higain_impl::xfader4_t<NV>, 
                                0>;
template <int NV>
using Space3 = parameter::plain<higain_impl::xfader6_t<NV>, 
                                0>;
template <int NV>
using LimMix = parameter::plain<higain_impl::xfader5_t<NV>, 
                                0>;
template <int NV>
using higain_t_plist = parameter::list<Thresh<NV>, 
                                       Attk<NV>, 
                                       Rel<NV>, 
                                       Ratio<NV>, 
                                       Input<NV>, 
                                       Out<NV>, 
                                       Scramble<NV>, 
                                       InputFilter<NV>, 
                                       InputFilterCut<NV>, 
                                       InputFilterq<NV>, 
                                       OutPutMode<NV>, 
                                       OutputCut<NV>, 
                                       OutputQ<NV>, 
                                       InMod<NV>, 
                                       OutMod<NV>, 
                                       verb<NV>, 
                                       scramblerate<NV>, 
                                       mix<NV>, 
                                       Sat<NV>, 
                                       Space2<NV>, 
                                       Space3<NV>, 
                                       over<NV>, 
                                       smooth1<NV>, 
                                       smooth2<NV>, 
                                       LimMix<NV>>;
}

template <int NV>
using higain_t_ = container::chain<higain_t_parameters::higain_t_plist<NV>, 
                                   wrap::fix<2, modchain_t<NV>>, 
                                   oversample_t<NV>>;

// =================================| Root node initialiser class |=================================

template <int NV> struct instance: public higain_impl::higain_t_<NV>
{
	
	struct metadata
	{
		static const int NumTables = 0;
		static const int NumSliderPacks = 0;
		static const int NumAudioFiles = 3;
		static const int NumFilters = 6;
		static const int NumDisplayBuffers = 1;
		
		SNEX_METADATA_ID(higain);
		SNEX_METADATA_NUM_CHANNELS(2);
		SNEX_METADATA_ENCODED_PARAMETERS(410)
		{
			0x005B, 0x0000, 0x5400, 0x7268, 0x7365, 0x0068, 0x0000, 0xC2C8, 
            0x0000, 0x0000, 0x9998, 0xC119, 0x833E, 0x40AD, 0xCCCD, 0x3DCC, 
            0x015B, 0x0000, 0x4100, 0x7474, 0x006B, 0x0000, 0x0000, 0x0000, 
            0x437A, 0x0000, 0x4342, 0x81A3, 0x3EDC, 0xCCCD, 0x3DCC, 0x025B, 
            0x0000, 0x5200, 0x6C65, 0x0000, 0x0000, 0x0000, 0x7A00, 0x0043, 
            0x0000, 0xA300, 0xDC81, 0xCD3E, 0xCCCC, 0x5B3D, 0x0003, 0x0000, 
            0x6152, 0x6974, 0x006F, 0x0000, 0x3F80, 0x0000, 0x4200, 0x999A, 
            0x4121, 0xF69D, 0x3E97, 0xCCCD, 0x3DCC, 0x045B, 0x0000, 0x4900, 
            0x706E, 0x7475, 0x0000, 0x0000, 0x0000, 0x8000, 0x6742, 0xAE66, 
            0x0041, 0x8000, 0xCD3F, 0xCCCC, 0x5B3D, 0x0005, 0x0000, 0x754F, 
            0x0074, 0x0000, 0xC2C8, 0x0000, 0x4140, 0x9997, 0xC099, 0x833E, 
            0x40AD, 0xCCCD, 0x3DCC, 0x065B, 0x0000, 0x5300, 0x7263, 0x6D61, 
            0x6C62, 0x0065, 0x0000, 0x0000, 0x0000, 0x3F80, 0x0000, 0x0000, 
            0x0000, 0x3F80, 0x0000, 0x0000, 0x075B, 0x0000, 0x4900, 0x706E, 
            0x7475, 0x6946, 0x746C, 0x7265, 0x0000, 0x8000, 0x003F, 0xC000, 
            0x0040, 0x4000, 0x0040, 0x8000, 0x003F, 0x8000, 0x5B3F, 0x0008, 
            0x0000, 0x6E49, 0x7570, 0x4674, 0x6C69, 0x6574, 0x4372, 0x7475, 
            0x0000, 0x0000, 0x0000, 0x8000, 0xB83F, 0x851E, 0x003E, 0x8000, 
            0x003F, 0x0000, 0x5B00, 0x0009, 0x0000, 0x6E49, 0x7570, 0x4674, 
            0x6C69, 0x6574, 0x7172, 0x0000, 0x0000, 0x0000, 0x8000, 0x663F, 
            0xE666, 0x183E, 0x8789, 0x003E, 0x0000, 0x5B00, 0x000A, 0x0000, 
            0x754F, 0x5074, 0x7475, 0x6F4D, 0x6564, 0x0000, 0x8000, 0x003F, 
            0xC000, 0x0040, 0x0000, 0x0040, 0x8000, 0x003F, 0x8000, 0x5B3F, 
            0x000B, 0x0000, 0x754F, 0x7074, 0x7475, 0x7543, 0x0074, 0x0000, 
            0x0000, 0x0000, 0x3F80, 0x47AE, 0x3DE1, 0x0000, 0x3F80, 0x0000, 
            0x0000, 0x0C5B, 0x0000, 0x4F00, 0x7475, 0x7570, 0x5174, 0x0000, 
            0x0000, 0x0000, 0x8000, 0xCD3F, 0x4CCC, 0x003E, 0x8000, 0x003F, 
            0x0000, 0x5B00, 0x000D, 0x0000, 0x6E49, 0x6F4D, 0x0064, 0x0000, 
            0xBF80, 0x0000, 0x3F80, 0xC28F, 0xBD75, 0x0000, 0x3F80, 0x0000, 
            0x0000, 0x0E5B, 0x0000, 0x4F00, 0x7475, 0x6F4D, 0x0064, 0x0000, 
            0xBF80, 0x0000, 0x3F80, 0x0000, 0x3F80, 0x0000, 0x3F80, 0x0000, 
            0x0000, 0x0F5B, 0x0000, 0x7600, 0x7265, 0x0062, 0x0000, 0x0000, 
            0x0000, 0x3F80, 0x3D71, 0x3F4A, 0x0000, 0x3F80, 0x0000, 0x0000, 
            0x105B, 0x0000, 0x7300, 0x7263, 0x6D61, 0x6C62, 0x7265, 0x7461, 
            0x0065, 0x0000, 0x0000, 0x0000, 0x4190, 0x0000, 0x4100, 0x0000, 
            0x3F80, 0x0000, 0x3F80, 0x115B, 0x0000, 0x6D00, 0x7869, 0x0000, 
            0x0000, 0x0000, 0x8000, 0x003F, 0x8000, 0x003F, 0x8000, 0x003F, 
            0x0000, 0x5B00, 0x0012, 0x0000, 0x6153, 0x0074, 0x0000, 0x0000, 
            0x0000, 0x3E80, 0x0000, 0x0000, 0x0000, 0x3F80, 0x0000, 0x0000, 
            0x135B, 0x0000, 0x5300, 0x6170, 0x6563, 0x0032, 0x0000, 0x0000, 
            0x0000, 0x3E80, 0x0000, 0x0000, 0x0000, 0x3F80, 0x0000, 0x0000, 
            0x145B, 0x0000, 0x5300, 0x6170, 0x6563, 0x0033, 0x0000, 0x0000, 
            0x0000, 0x3E80, 0x0000, 0x0000, 0x0000, 0x3F80, 0x0000, 0x0000, 
            0x155B, 0x0000, 0x6F00, 0x6576, 0x0072, 0x0000, 0x3F80, 0x0000, 
            0x40A0, 0x0000, 0x4040, 0x0000, 0x3F80, 0x0000, 0x3F80, 0x165B, 
            0x0000, 0x7300, 0x6F6D, 0x746F, 0x3168, 0x0000, 0x0000, 0x0000, 
            0x8000, 0x593F, 0x1AC8, 0x003E, 0x8000, 0x003F, 0x0000, 0x5B00, 
            0x0017, 0x0000, 0x6D73, 0x6F6F, 0x6874, 0x0032, 0x0000, 0x0000, 
            0x0000, 0x3F80, 0x642D, 0x3E1D, 0x0000, 0x3F80, 0x0000, 0x0000, 
            0x185B, 0x0000, 0x4C00, 0x6D69, 0x694D, 0x0078, 0x0000, 0x0000, 
            0x0000, 0x3F80, 0x0000, 0x0000, 0x0000, 0x3F80, 0x0000, 0x0000, 
            0x0000, 0x0000
		};
	};
	
	instance()
	{
		// Node References -------------------------------------------------------------------------
		
		auto& modchain = this->getT(0);                                                      // higain_impl::modchain_t<NV>
		auto& split = this->getT(0).getT(0);                                                 // higain_impl::split_t<NV>
		auto& chain2 = this->getT(0).getT(0).getT(0);                                        // higain_impl::chain2_t<NV>
		auto& pma1 = this->getT(0).getT(0).getT(0).getT(0);                                  // higain_impl::pma1_t<NV>
		auto& chain1 = this->getT(0).getT(0).getT(1);                                        // higain_impl::chain1_t<NV>
		auto& pma = this->getT(0).getT(0).getT(1).getT(0);                                   // higain_impl::pma_t<NV>
		auto& oversample = this->getT(1);                                                    // higain_impl::oversample_t<NV>
		auto& chain7 = this->getT(1).getT(0);                                                // higain_impl::chain7_t<NV>
		auto& xfader = this->getT(1).getT(0).getT(0);                                        // higain_impl::xfader_t<NV>
		auto& split2 = this->getT(1).getT(0).getT(1);                                        // higain_impl::split2_t<NV>
		auto& chain8 = this->getT(1).getT(0).getT(1).getT(0);                                // higain_impl::chain8_t<NV>
		auto& gain2 = this->getT(1).getT(0).getT(1).getT(0).getT(0);                         // core::gain<NV>
		auto& chain9 = this->getT(1).getT(0).getT(1).getT(1);                                // higain_impl::chain9_t<NV>
		auto& chain14 = this->getT(1).getT(0).getT(1).getT(1).getT(0);                       // higain_impl::chain14_t<NV>
		auto& expr = this->getT(1).getT(0).getT(1).getT(1).getT(0).getT(0);                  // math::expr<NV, custom::expr>
		auto& xfader2 = this->getT(1).getT(0).getT(1).getT(1).getT(0).getT(1);               // higain_impl::xfader2_t<NV>
		auto& split1 = this->getT(1).getT(0).getT(1).getT(1).getT(0).getT(2);                // higain_impl::split1_t<NV>
		auto& chain11 = this->getT(1).getT(0).getT(1).getT(1).getT(0).getT(2).getT(0);       // higain_impl::chain11_t<NV>
		auto& gain15 = this->getT(1).getT(0).getT(1).getT(1).                                // core::gain<NV>
                       getT(0).getT(2).getT(0).getT(0);
		auto& chain12 = this->getT(1).getT(0).getT(1).getT(1).getT(0).getT(2).getT(1);       // higain_impl::chain12_t<NV>
		auto& reverb = this->getT(1).getT(0).getT(1).getT(1).                                // fx::reverb
                       getT(0).getT(2).getT(1).getT(0);
		auto& gain14 = this->getT(1).getT(0).getT(1).getT(1).                                // core::gain<NV>
                       getT(0).getT(2).getT(1).getT(1);
		auto& branch4 = this->getT(1).getT(0).getT(1).getT(1).getT(1);                       // higain_impl::branch4_t<NV>
		auto& chain20 = this->getT(1).getT(0).getT(1).getT(1).getT(1).getT(0);               // higain_impl::chain20_t<NV>
		auto& svf1 = this->getT(1).getT(0).getT(1).getT(1).getT(1).getT(0).getT(0);          // higain_impl::svf1_t<NV>
		auto& chain21 = this->getT(1).getT(0).getT(1).getT(1).getT(1).getT(1);               // higain_impl::chain21_t<NV>
		auto& svf7 = this->getT(1).getT(0).getT(1).getT(1).getT(1).getT(1).getT(0);          // higain_impl::svf7_t<NV>
		auto& chain22 = this->getT(1).getT(0).getT(1).getT(1).getT(1).getT(2);               // higain_impl::chain22_t<NV>
		auto& svf8 = this->getT(1).getT(0).getT(1).getT(1).getT(1).getT(2).getT(0);          // higain_impl::svf8_t<NV>
		auto& chain30 = this->getT(1).getT(0).getT(1).getT(1).getT(1).getT(3);               // higain_impl::chain30_t<NV>
		auto& allpass1 = this->getT(1).getT(0).getT(1).getT(1).getT(1).getT(3).getT(0);      // filters::allpass<NV>
		auto& chain23 = this->getT(1).getT(0).getT(1).getT(1).getT(1).getT(4);               // higain_impl::chain23_t<NV>
		auto& receive1 = this->getT(1).getT(0).getT(1).getT(1).getT(1).getT(4).getT(0);      // routing::receive<stereo_cable>
		auto& snex_node1 = this->getT(1).getT(0).getT(1).getT(1).getT(1).getT(4).getT(1);    // sampleandramp<NV>
		auto& send1 = this->getT(1).getT(0).getT(1).getT(1).getT(1).getT(4).getT(2);         // routing::send<stereo_cable>
		auto& chain35 = this->getT(1).getT(0).getT(1).getT(1).getT(1).getT(5);               // higain_impl::chain35_t<NV>
		auto& receive4 = this->getT(1).getT(0).getT(1).getT(1).getT(1).getT(5).getT(0);      // routing::receive<stereo_cable>
		auto& faust = this->getT(1).getT(0).getT(1).getT(1).getT(1).getT(5).getT(1);         // project::Wavefolder<NV>
		auto& send4 = this->getT(1).getT(0).getT(1).getT(1).getT(1).getT(5).getT(2);         // routing::send<stereo_cable>
		auto& gain = this->getT(1).getT(0).getT(1).getT(1).getT(2);                          // core::gain<NV>
		auto& haas = this->getT(1).getT(0).getT(1).getT(1).getT(3);                          // wrap::no_process<fx::haas<NV>>
		auto& split4 = this->getT(1).getT(0).getT(1).getT(1).getT(4);                        // higain_impl::split4_t<NV>
		auto& chain10 = this->getT(1).getT(0).getT(1).getT(1).getT(4).getT(0);               // higain_impl::chain10_t<NV>
		auto& xfader5 = this->getT(1).getT(0).getT(1).getT(1).getT(4).getT(0).getT(0);       // higain_impl::xfader5_t<NV>
		auto& split7 = this->getT(1).getT(0).getT(1).getT(1).getT(4).getT(0).getT(1);        // higain_impl::split7_t<NV>
		auto& chain31 = this->getT(1).getT(0).getT(1).getT(1).                               // higain_impl::chain31_t<NV>
                        getT(4).getT(0).getT(1).getT(0);
		auto& gain8 = this->getT(1).getT(0).getT(1).getT(1).                                 // core::gain<NV>
                      getT(4).getT(0).getT(1).getT(0).
                      getT(0);
		auto& chain37 = this->getT(1).getT(0).getT(1).getT(1).                               // higain_impl::chain37_t<NV>
                        getT(4).getT(0).getT(1).getT(1);
		auto& limiter = this->getT(1).getT(0).getT(1).getT(1).                               // higain_impl::limiter_t<NV>
                        getT(4).getT(0).getT(1).getT(1).
                        getT(0);
		auto& gain7 = this->getT(1).getT(0).getT(1).getT(1).                                 // core::gain<NV>
                      getT(4).getT(0).getT(1).getT(1).
                      getT(1);
		auto& chain13 = this->getT(1).getT(0).getT(1).getT(1).getT(4).getT(1);               // higain_impl::chain13_t<NV>
		auto& tempo_sync = this->getT(1).getT(0).getT(1).getT(1).getT(4).getT(1).getT(0);    // higain_impl::tempo_sync_t<NV>
		auto& converter = this->getT(1).getT(0).getT(1).getT(1).getT(4).getT(1).getT(1);     // higain_impl::converter_t<NV>
		auto& clear2 = this->getT(1).getT(0).getT(1).getT(1).getT(4).getT(1).getT(2);        // math::clear<NV>
		auto& oscillator = this->getT(1).getT(0).getT(1).getT(1).getT(4).getT(1).getT(3);    // higain_impl::oscillator_t<NV>
		auto& sampleandhold = this->getT(1).getT(0).getT(1).getT(1).getT(4).getT(1).getT(4); // fx::sampleandhold<NV>
		auto& mod_inv = this->getT(1).getT(0).getT(1).getT(1).getT(4).getT(1).getT(5);       // wrap::no_process<math::mod_inv<NV>>
		auto& rect = this->getT(1).getT(0).getT(1).getT(1).getT(4).getT(1).getT(6);          // math::rect<NV>
		auto& peak = this->getT(1).getT(0).getT(1).getT(1).getT(4).getT(1).getT(7);          // higain_impl::peak_t<NV>
		auto& clear1 = this->getT(1).getT(0).getT(1).getT(1).getT(4).getT(1).getT(8);        // math::clear<NV>
		auto& chain26 = this->getT(1).getT(0).getT(1).getT(1).getT(5);                       // higain_impl::chain26_t<NV>
		auto& xfader4 = this->getT(1).getT(0).getT(1).getT(1).getT(5).getT(0);               // higain_impl::xfader4_t<NV>
		auto& split6 = this->getT(1).getT(0).getT(1).getT(1).getT(5).getT(1);                // higain_impl::split6_t<NV>
		auto& chain27 = this->getT(1).getT(0).getT(1).getT(1).getT(5).getT(1).getT(0);       // higain_impl::chain27_t<NV>
		auto& gain18 = this->getT(1).getT(0).getT(1).getT(1).                                // core::gain<NV>
                       getT(5).getT(1).getT(0).getT(0);
		auto& chain28 = this->getT(1).getT(0).getT(1).getT(1).getT(5).getT(1).getT(1);    // higain_impl::chain28_t<NV>
		auto& convolution2 = this->getT(1).getT(0).getT(1).getT(1).                       // higain_impl::convolution2_t
                             getT(5).getT(1).getT(1).getT(0);
		auto& gain19 = this->getT(1).getT(0).getT(1).getT(1).                             // core::gain<NV>
                       getT(5).getT(1).getT(1).getT(1);
		auto& xfader1 = this->getT(1).getT(0).getT(1).getT(1).getT(6);                    // higain_impl::xfader1_t<NV>
		auto& split3 = this->getT(1).getT(0).getT(1).getT(1).getT(7);                     // higain_impl::split3_t<NV>
		auto& chain5 = this->getT(1).getT(0).getT(1).getT(1).getT(7).getT(0);             // higain_impl::chain5_t<NV>
		auto& gain4 = this->getT(1).getT(0).getT(1).getT(1).getT(7).getT(0).getT(0);      // core::gain<NV>
		auto& chain6 = this->getT(1).getT(0).getT(1).getT(1).getT(7).getT(1);             // higain_impl::chain6_t<NV>
		auto& chain = this->getT(1).getT(0).getT(1).getT(1).getT(7).getT(1).getT(0);      // higain_impl::chain_t<NV>
		auto& branch = this->getT(1).getT(0).getT(1).getT(1).                             // higain_impl::branch_t<NV>
                       getT(7).getT(1).getT(0).getT(0);
		auto& chain3 = this->getT(1).getT(0).getT(1).getT(1).                             // higain_impl::chain3_t
                       getT(7).getT(1).getT(0).getT(0).
                       getT(0);
		auto& chain4 = this->getT(1).getT(0).getT(1).getT(1).                             // higain_impl::chain4_t<NV>
                       getT(7).getT(1).getT(0).getT(0).
                       getT(1);
		auto& clear = this->getT(1).getT(0).getT(1).getT(1).getT(7).                      // math::clear<NV>
                      getT(1).getT(0).getT(0).getT(1).getT(0);
		auto& gain5 = this->getT(1).getT(0).getT(1).getT(1).                              // core::gain<NV>
                      getT(7).getT(1).getT(0).getT(1);
		auto& branch3 = this->getT(1).getT(0).getT(1).getT(1).getT(8);                    // higain_impl::branch3_t<NV>
		auto& chain16 = this->getT(1).getT(0).getT(1).getT(1).getT(8).getT(0);            // higain_impl::chain16_t<NV>
		auto& svf = this->getT(1).getT(0).getT(1).getT(1).getT(8).getT(0).getT(0);        // higain_impl::svf_t<NV>
		auto& chain18 = this->getT(1).getT(0).getT(1).getT(1).getT(8).getT(1);            // higain_impl::chain18_t<NV>
		auto& svf4 = this->getT(1).getT(0).getT(1).getT(1).getT(8).getT(1).getT(0);       // higain_impl::svf4_t<NV>
		auto& chain17 = this->getT(1).getT(0).getT(1).getT(1).getT(8).getT(2);            // higain_impl::chain17_t<NV>
		auto& svf3 = this->getT(1).getT(0).getT(1).getT(1).getT(8).getT(2).getT(0);       // higain_impl::svf3_t<NV>
		auto& chain29 = this->getT(1).getT(0).getT(1).getT(1).getT(8).getT(3);            // higain_impl::chain29_t<NV>
		auto& allpass = this->getT(1).getT(0).getT(1).getT(1).getT(8).getT(3).getT(0);    // filters::allpass<NV>
		auto& chain19 = this->getT(1).getT(0).getT(1).getT(1).getT(8).getT(4);            // higain_impl::chain19_t<NV>
		auto& receive = this->getT(1).getT(0).getT(1).getT(1).getT(8).getT(4).getT(0);    // routing::receive<stereo_cable>
		auto& snex_node2 = this->getT(1).getT(0).getT(1).getT(1).getT(8).getT(4).getT(1); // sampleandramp<NV>
		auto& send = this->getT(1).getT(0).getT(1).getT(1).getT(8).getT(4).getT(2);       // routing::send<stereo_cable>
		auto& chain36 = this->getT(1).getT(0).getT(1).getT(1).getT(8).getT(5);            // higain_impl::chain36_t<NV>
		auto& receive3 = this->getT(1).getT(0).getT(1).getT(1).getT(8).getT(5).getT(0);   // routing::receive<stereo_cable>
		auto& faust1 = this->getT(1).getT(0).getT(1).getT(1).getT(8).getT(5).getT(1);     // project::Wavefolder<NV>
		auto& send3 = this->getT(1).getT(0).getT(1).getT(1).getT(8).getT(5).getT(2);      // routing::send<stereo_cable>
		auto& chain32 = this->getT(1).getT(0).getT(1).getT(1).getT(9);                    // higain_impl::chain32_t<NV>
		auto& xfader6 = this->getT(1).getT(0).getT(1).getT(1).getT(9).getT(0);            // higain_impl::xfader6_t<NV>
		auto& split8 = this->getT(1).getT(0).getT(1).getT(1).getT(9).getT(1);             // higain_impl::split8_t<NV>
		auto& chain33 = this->getT(1).getT(0).getT(1).getT(1).getT(9).getT(1).getT(0);    // higain_impl::chain33_t<NV>
		auto& gain22 = this->getT(1).getT(0).getT(1).getT(1).                             // core::gain<NV>
                       getT(9).getT(1).getT(0).getT(0);
		auto& chain34 = this->getT(1).getT(0).getT(1).getT(1).getT(9).getT(1).getT(1); // higain_impl::chain34_t<NV>
		auto& convolution3 = this->getT(1).getT(0).getT(1).getT(1).                    // higain_impl::convolution3_t
                             getT(9).getT(1).getT(1).getT(0);
		auto& gain23 = this->getT(1).getT(0).getT(1).getT(1).         // core::gain<NV>
                       getT(9).getT(1).getT(1).getT(1);
		auto& gain1 = this->getT(1).getT(0).getT(1).getT(1).getT(10); // core::gain<NV>
		auto& gain3 = this->getT(1).getT(0).getT(1).getT(1).getT(11); // core::gain<NV>
		
		// Parameter Connections -------------------------------------------------------------------
		
		this->getParameterT(0).connectT(0, limiter); // Thresh -> limiter::Threshhold
		
		this->getParameterT(1).connectT(0, limiter); // Attk -> limiter::Attack
		
		this->getParameterT(2).connectT(0, limiter); // Rel -> limiter::Release
		
		this->getParameterT(3).connectT(0, limiter); // Ratio -> limiter::Ratio
		
		this->getParameterT(4).connectT(0, gain); // Input -> gain::Gain
		
		this->getParameterT(5).connectT(0, gain1); // Out -> gain1::Gain
		
		this->getParameterT(6).connectT(0, xfader1); // Scramble -> xfader1::Value
		
		this->getParameterT(7).connectT(0, branch4); // InputFilter -> branch4::Index
		
		this->getParameterT(8).connectT(0, pma1); // InputFilterCut -> pma1::Add
		
		auto& InputFilterq_p = this->getParameterT(9);
		InputFilterq_p.connectT(0, svf1);     // InputFilterq -> svf1::Q
		InputFilterq_p.connectT(1, receive1); // InputFilterq -> receive1::Feedback
		InputFilterq_p.connectT(2, svf8);     // InputFilterq -> svf8::Q
		InputFilterq_p.connectT(3, svf7);     // InputFilterq -> svf7::Q
		InputFilterq_p.connectT(4, allpass1); // InputFilterq -> allpass1::Q
		InputFilterq_p.connectT(5, receive4); // InputFilterq -> receive4::Feedback
		
		this->getParameterT(10).connectT(0, branch3); // OutPutMode -> branch3::Index
		
		this->getParameterT(11).connectT(0, pma); // OutputCut -> pma::Add
		
		auto& OutputQ_p = this->getParameterT(12);
		OutputQ_p.connectT(0, svf);      // OutputQ -> svf::Q
		OutputQ_p.connectT(1, svf4);     // OutputQ -> svf4::Q
		OutputQ_p.connectT(2, svf3);     // OutputQ -> svf3::Q
		OutputQ_p.connectT(3, receive);  // OutputQ -> receive::Feedback
		OutputQ_p.connectT(4, allpass);  // OutputQ -> allpass::Q
		OutputQ_p.connectT(5, receive3); // OutputQ -> receive3::Feedback
		
		this->getParameterT(13).connectT(0, pma1); // InMod -> pma1::Multiply
		
		this->getParameterT(14).connectT(0, pma); // OutMod -> pma::Multiply
		
		this->getParameterT(15).connectT(0, xfader2); // verb -> xfader2::Value
		
		auto& scramblerate_p = this->getParameterT(16);
		scramblerate_p.connectT(0, tempo_sync); // scramblerate -> tempo_sync::Tempo
		scramblerate_p.connectT(1, faust1);     // scramblerate -> faust1::windowsamples
		scramblerate_p.connectT(2, faust1);     // scramblerate -> faust1::xfadesamples
		scramblerate_p.connectT(3, faust);      // scramblerate -> faust::windowsamples
		scramblerate_p.connectT(4, faust);      // scramblerate -> faust::xfadesamples
		
		this->getParameterT(17).connectT(0, xfader); // mix -> xfader::Value
		
		this->getParameterT(18).connectT(0, expr); // Sat -> expr::Value
		
		this->getParameterT(19).connectT(0, xfader4); // Space2 -> xfader4::Value
		
		this->getParameterT(20).connectT(0, xfader6); // Space3 -> xfader6::Value
		
		this->getParameterT(21).connectT(0, oversample); // over -> oversample::Oversampling
		
		auto& smooth1_p = this->getParameterT(22);
		smooth1_p.connectT(0, svf1); // smooth1 -> svf1::Smoothing
		smooth1_p.connectT(1, svf7); // smooth1 -> svf7::Smoothing
		smooth1_p.connectT(2, svf8); // smooth1 -> svf8::Smoothing
		
		auto& smooth2_p = this->getParameterT(23);
		smooth2_p.connectT(0, svf);  // smooth2 -> svf::Smoothing
		smooth2_p.connectT(1, svf4); // smooth2 -> svf4::Smoothing
		smooth2_p.connectT(2, svf3); // smooth2 -> svf3::Smoothing
		
		this->getParameterT(24).connectT(0, xfader5); // LimMix -> xfader5::Value
		
		// Modulation Connections ------------------------------------------------------------------
		
		pma1.getWrappedObject().getParameter().connectT(0, svf1);       // pma1 -> svf1::Frequency
		pma1.getWrappedObject().getParameter().connectT(1, svf7);       // pma1 -> svf7::Frequency
		pma1.getWrappedObject().getParameter().connectT(2, svf8);       // pma1 -> svf8::Frequency
		pma1.getWrappedObject().getParameter().connectT(3, snex_node1); // pma1 -> snex_node1::Parameter
		pma1.getWrappedObject().getParameter().connectT(4, faust);      // pma1 -> faust::shiftsemitones
		pma1.getWrappedObject().getParameter().connectT(5, allpass1);   // pma1 -> allpass1::Frequency
		pma.getWrappedObject().getParameter().connectT(0, svf);         // pma -> svf::Frequency
		pma.getWrappedObject().getParameter().connectT(1, svf4);        // pma -> svf4::Frequency
		pma.getWrappedObject().getParameter().connectT(2, svf3);        // pma -> svf3::Frequency
		pma.getWrappedObject().getParameter().connectT(3, snex_node2);  // pma -> snex_node2::Parameter
		pma.getWrappedObject().getParameter().connectT(4, allpass);     // pma -> allpass::Frequency
		pma.getWrappedObject().getParameter().connectT(5, faust1);      // pma -> faust1::shiftsemitones
		auto& xfader_p = xfader.getWrappedObject().getParameter();
		xfader_p.getParameterT(0).connectT(0, gain2); // xfader -> gain2::Gain
		xfader_p.getParameterT(1).connectT(0, gain3); // xfader -> gain3::Gain
		auto& xfader2_p = xfader2.getWrappedObject().getParameter();
		xfader2_p.getParameterT(0).connectT(0, gain15); // xfader2 -> gain15::Gain
		xfader2_p.getParameterT(1).connectT(0, gain14); // xfader2 -> gain14::Gain
		auto& xfader5_p = xfader5.getWrappedObject().getParameter();
		xfader5_p.getParameterT(0).connectT(0, gain8);                          // xfader5 -> gain8::Gain
		xfader5_p.getParameterT(1).connectT(0, gain7);                          // xfader5 -> gain7::Gain
		limiter.getParameter().connectT(0, pma);                                // limiter -> pma::Value
		limiter.getParameter().connectT(1, pma1);                               // limiter -> pma1::Value
		converter.getWrappedObject().getParameter().connectT(0, sampleandhold); // converter -> sampleandhold::Counter
		tempo_sync.getParameter().connectT(0, converter);                       // tempo_sync -> converter::Value
		peak.getParameter().connectT(0, branch);                                // peak -> branch::Index
		auto& xfader4_p = xfader4.getWrappedObject().getParameter();
		xfader4_p.getParameterT(0).connectT(0, gain18); // xfader4 -> gain18::Gain
		xfader4_p.getParameterT(1).connectT(0, gain19); // xfader4 -> gain19::Gain
		auto& xfader1_p = xfader1.getWrappedObject().getParameter();
		xfader1_p.getParameterT(0).connectT(0, gain4); // xfader1 -> gain4::Gain
		xfader1_p.getParameterT(1).connectT(0, gain5); // xfader1 -> gain5::Gain
		auto& xfader6_p = xfader6.getWrappedObject().getParameter();
		xfader6_p.getParameterT(0).connectT(0, gain22); // xfader6 -> gain22::Gain
		xfader6_p.getParameterT(1).connectT(0, gain23); // xfader6 -> gain23::Gain
		
		// Send Connections ------------------------------------------------------------------------
		
		send1.connect(receive1);
		send4.connect(receive4);
		send.connect(receive);
		send3.connect(receive3);
		
		// Default Values --------------------------------------------------------------------------
		
		; // pma1::Value is automated
		; // pma1::Multiply is automated
		; // pma1::Add is automated
		
		; // pma::Value is automated
		; // pma::Multiply is automated
		; // pma::Add is automated
		
		;                                // oversample::Oversampling is automated
		oversample.setParameterT(1, 0.); // container::chain::FilterType
		
		; // xfader::Value is automated
		
		;                             // gain2::Gain is automated
		gain2.setParameterT(1, 20.);  // core::gain::Smoothing
		gain2.setParameterT(2, -15.); // core::gain::ResetValue
		
		; // expr::Value is automated
		
		; // xfader2::Value is automated
		
		;                             // gain15::Gain is automated
		gain15.setParameterT(1, 20.); // core::gain::Smoothing
		gain15.setParameterT(2, 0.);  // core::gain::ResetValue
		
		reverb.setParameterT(0, 0.); // fx::reverb::Damping
		reverb.setParameterT(1, 1.); // fx::reverb::Width
		reverb.setParameterT(2, 1.); // fx::reverb::Size
		
		;                             // gain14::Gain is automated
		gain14.setParameterT(1, 20.); // core::gain::Smoothing
		gain14.setParameterT(2, 0.);  // core::gain::ResetValue
		
		; // branch4::Index is automated
		
		;                          // svf1::Frequency is automated
		;                          // svf1::Q is automated
		svf1.setParameterT(2, 0.); // filters::svf::Gain
		;                          // svf1::Smoothing is automated
		svf1.setParameterT(4, 0.); // filters::svf::Mode
		svf1.setParameterT(5, 1.); // filters::svf::Enabled
		
		;                          // svf7::Frequency is automated
		;                          // svf7::Q is automated
		svf7.setParameterT(2, 0.); // filters::svf::Gain
		;                          // svf7::Smoothing is automated
		svf7.setParameterT(4, 1.); // filters::svf::Mode
		svf7.setParameterT(5, 1.); // filters::svf::Enabled
		
		;                           // svf8::Frequency is automated
		;                           // svf8::Q is automated
		svf8.setParameterT(2, 18.); // filters::svf::Gain
		;                           // svf8::Smoothing is automated
		svf8.setParameterT(4, 2.);  // filters::svf::Mode
		svf8.setParameterT(5, 1.);  // filters::svf::Enabled
		
		;                                // allpass1::Frequency is automated
		;                                // allpass1::Q is automated
		allpass1.setParameterT(2, 0.);   // filters::allpass::Gain
		allpass1.setParameterT(3, 0.01); // filters::allpass::Smoothing
		allpass1.setParameterT(4, 0.);   // filters::allpass::Mode
		allpass1.setParameterT(5, 1.);   // filters::allpass::Enabled
		
		; // receive1::Feedback is automated
		
		; // snex_node1::Parameter is automated
		
		; // receive4::Feedback is automated
		
		; // faust::shiftsemitones is automated
		; // faust::windowsamples is automated
		; // faust::xfadesamples is automated
		
		;                           // gain::Gain is automated
		gain.setParameterT(1, 20.); // core::gain::Smoothing
		gain.setParameterT(2, 0.);  // core::gain::ResetValue
		
		haas.setParameterT(0, 1.49012e-08); // fx::haas::Position
		
		; // xfader5::Value is automated
		
		;                            // gain8::Gain is automated
		gain8.setParameterT(1, 20.); // core::gain::Smoothing
		gain8.setParameterT(2, 0.);  // core::gain::ResetValue
		
		;                             // limiter::Threshhold is automated
		;                             // limiter::Attack is automated
		;                             // limiter::Release is automated
		;                             // limiter::Ratio is automated
		limiter.setParameterT(4, 0.); // dynamics::limiter::Sidechain
		
		;                            // gain7::Gain is automated
		gain7.setParameterT(1, 20.); // core::gain::Smoothing
		gain7.setParameterT(2, 0.);  // core::gain::ResetValue
		
		;                                // tempo_sync::Tempo is automated
		tempo_sync.setParameterT(1, 1.); // control::tempo_sync::Multiplier
		tempo_sync.setParameterT(2, 1.); // control::tempo_sync::Enabled
		tempo_sync.setParameterT(3, 0.); // control::tempo_sync::UnsyncedTime
		
		; // converter::Value is automated
		
		clear2.setParameterT(0, 0.); // math::clear::Value
		
		oscillator.setParameterT(0, 4.);    // core::oscillator::Mode
		oscillator.setParameterT(1, 107.9); // core::oscillator::Frequency
		oscillator.setParameterT(2, 1.);    // core::oscillator::FreqRatio
		oscillator.setParameterT(3, 1.);    // core::oscillator::Gate
		oscillator.setParameterT(4, 0.);    // core::oscillator::Phase
		oscillator.setParameterT(5, 1.);    // core::oscillator::Gain
		
		; // sampleandhold::Counter is automated
		
		mod_inv.setParameterT(0, 0.); // math::mod_inv::Value
		
		rect.setParameterT(0, 0.); // math::rect::Value
		
		clear1.setParameterT(0, 0.); // math::clear::Value
		
		; // xfader4::Value is automated
		
		;                             // gain18::Gain is automated
		gain18.setParameterT(1, 20.); // core::gain::Smoothing
		gain18.setParameterT(2, 0.);  // core::gain::ResetValue
		
		convolution2.setParameterT(0, 1.);     // filters::convolution::Gate
		convolution2.setParameterT(1, 0.);     // filters::convolution::Predelay
		convolution2.setParameterT(2, 0.);     // filters::convolution::Damping
		convolution2.setParameterT(3, 20000.); // filters::convolution::HiCut
		convolution2.setParameterT(4, 1.);     // filters::convolution::Multithread
		
		;                             // gain19::Gain is automated
		gain19.setParameterT(1, 20.); // core::gain::Smoothing
		gain19.setParameterT(2, 0.);  // core::gain::ResetValue
		
		; // xfader1::Value is automated
		
		;                            // gain4::Gain is automated
		gain4.setParameterT(1, 20.); // core::gain::Smoothing
		gain4.setParameterT(2, 0.);  // core::gain::ResetValue
		
		; // branch::Index is automated
		
		clear.setParameterT(0, 0.); // math::clear::Value
		
		;                            // gain5::Gain is automated
		gain5.setParameterT(1, 20.); // core::gain::Smoothing
		gain5.setParameterT(2, 0.);  // core::gain::ResetValue
		
		; // branch3::Index is automated
		
		;                         // svf::Frequency is automated
		;                         // svf::Q is automated
		svf.setParameterT(2, 0.); // filters::svf::Gain
		;                         // svf::Smoothing is automated
		svf.setParameterT(4, 0.); // filters::svf::Mode
		svf.setParameterT(5, 1.); // filters::svf::Enabled
		
		;                            // svf4::Frequency is automated
		;                            // svf4::Q is automated
		svf4.setParameterT(2, -0.3); // filters::svf::Gain
		;                            // svf4::Smoothing is automated
		svf4.setParameterT(4, 1.);   // filters::svf::Mode
		svf4.setParameterT(5, 1.);   // filters::svf::Enabled
		
		;                          // svf3::Frequency is automated
		;                          // svf3::Q is automated
		svf3.setParameterT(2, 0.); // filters::svf::Gain
		;                          // svf3::Smoothing is automated
		svf3.setParameterT(4, 2.); // filters::svf::Mode
		svf3.setParameterT(5, 1.); // filters::svf::Enabled
		
		;                               // allpass::Frequency is automated
		;                               // allpass::Q is automated
		allpass.setParameterT(2, 0.);   // filters::allpass::Gain
		allpass.setParameterT(3, 0.01); // filters::allpass::Smoothing
		allpass.setParameterT(4, 0.);   // filters::allpass::Mode
		allpass.setParameterT(5, 1.);   // filters::allpass::Enabled
		
		; // receive::Feedback is automated
		
		; // snex_node2::Parameter is automated
		
		; // receive3::Feedback is automated
		
		; // faust1::shiftsemitones is automated
		; // faust1::windowsamples is automated
		; // faust1::xfadesamples is automated
		
		; // xfader6::Value is automated
		
		;                             // gain22::Gain is automated
		gain22.setParameterT(1, 20.); // core::gain::Smoothing
		gain22.setParameterT(2, 0.);  // core::gain::ResetValue
		
		convolution3.setParameterT(0, 1.);     // filters::convolution::Gate
		convolution3.setParameterT(1, 0.);     // filters::convolution::Predelay
		convolution3.setParameterT(2, 0.);     // filters::convolution::Damping
		convolution3.setParameterT(3, 20000.); // filters::convolution::HiCut
		convolution3.setParameterT(4, 1.);     // filters::convolution::Multithread
		
		;                             // gain23::Gain is automated
		gain23.setParameterT(1, 20.); // core::gain::Smoothing
		gain23.setParameterT(2, 0.);  // core::gain::ResetValue
		
		;                            // gain1::Gain is automated
		gain1.setParameterT(1, 20.); // core::gain::Smoothing
		gain1.setParameterT(2, 0.);  // core::gain::ResetValue
		
		;                            // gain3::Gain is automated
		gain3.setParameterT(1, 20.); // core::gain::Smoothing
		gain3.setParameterT(2, 0.);  // core::gain::ResetValue
		
		this->setParameterT(0, -9.6);
		this->setParameterT(1, 194.);
		this->setParameterT(2, 0.);
		this->setParameterT(3, 10.1);
		this->setParameterT(4, 21.8);
		this->setParameterT(5, -4.8);
		this->setParameterT(6, 0.);
		this->setParameterT(7, 3.);
		this->setParameterT(8, 0.26);
		this->setParameterT(9, 0.45);
		this->setParameterT(10, 2.);
		this->setParameterT(11, 0.11);
		this->setParameterT(12, 0.2);
		this->setParameterT(13, -0.06);
		this->setParameterT(14, 1.);
		this->setParameterT(15, 0.79);
		this->setParameterT(16, 8.);
		this->setParameterT(17, 1.);
		this->setParameterT(18, 0.);
		this->setParameterT(19, 0.);
		this->setParameterT(20, 0.);
		this->setParameterT(21, 3.);
		this->setParameterT(22, 0.151155);
		this->setParameterT(23, 0.153702);
		this->setParameterT(24, 0.);
		this->setExternalData({}, -1);
	}
	~instance() override
	{
		// Cleanup external data references --------------------------------------------------------
		
		this->setExternalData({}, -1);
	}
	
	static constexpr bool isPolyphonic() { return NV > 1; };
	
	static constexpr bool isProcessingHiseEvent() { return true; };
	
	static constexpr bool hasTail() { return true; };
	
	static constexpr bool isSuspendedOnSilence() { return false; };
	
	void setExternalData(const ExternalData& b, int index)
	{
		// External Data Connections ---------------------------------------------------------------
		
		this->getT(1).getT(0).getT(1).getT(1).getT(1).getT(0).getT(0).setExternalData(b, index); // higain_impl::svf1_t<NV>
		this->getT(1).getT(0).getT(1).getT(1).getT(1).getT(1).getT(0).setExternalData(b, index); // higain_impl::svf7_t<NV>
		this->getT(1).getT(0).getT(1).getT(1).getT(1).getT(2).getT(0).setExternalData(b, index); // higain_impl::svf8_t<NV>
		this->getT(1).getT(0).getT(1).getT(1).                                                   // higain_impl::limiter_t<NV>
        getT(4).getT(0).getT(1).getT(1).
        getT(0).setExternalData(b, index);
		this->getT(1).getT(0).getT(1).getT(1).getT(4).getT(1).getT(3).setExternalData(b, index); // higain_impl::oscillator_t<NV>
		this->getT(1).getT(0).getT(1).getT(1).getT(4).getT(1).getT(7).setExternalData(b, index); // higain_impl::peak_t<NV>
		this->getT(1).getT(0).getT(1).getT(1).                                                   // higain_impl::convolution2_t
        getT(5).getT(1).getT(1).getT(0).setExternalData(b, index);
		this->getT(1).getT(0).getT(1).getT(1).getT(8).getT(0).getT(0).setExternalData(b, index); // higain_impl::svf_t<NV>
		this->getT(1).getT(0).getT(1).getT(1).getT(8).getT(1).getT(0).setExternalData(b, index); // higain_impl::svf4_t<NV>
		this->getT(1).getT(0).getT(1).getT(1).getT(8).getT(2).getT(0).setExternalData(b, index); // higain_impl::svf3_t<NV>
		this->getT(1).getT(0).getT(1).getT(1).                                                   // higain_impl::convolution3_t
        getT(9).getT(1).getT(1).getT(0).setExternalData(b, index);
	}
};
}

#undef getT
#undef connectT
#undef setParameterT
#undef setParameterWT
#undef getParameterT
// ======================================| Public Definition |======================================

namespace project
{
// polyphonic template declaration

template <int NV>
using higain = wrap::node<higain_impl::instance<NV>>;
}


