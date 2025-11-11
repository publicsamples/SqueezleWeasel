Content.makeFrontInterface(500, 430);

Engine.loadFontAs("{PROJECT_FOLDER}Montserrat-Medium.ttf", "Montserrat");

const var ScriptIR = Synth.getAudioSampleProcessor("Script FX1");

const slot1 = ScriptIR.getAudioFile(0);
const slot2 = ScriptIR.getAudioFile(1);
const slot3 = ScriptIR.getAudioFile(2);


inline function onEffectControl(component, value)
{
slot1.loadFile("{PROJECT_FOLDER}Resonant Ping 12.wav");  
slot2.loadFile("{PROJECT_FOLDER}Cruise Ship Cabin HVAC Ceiling Plate.wav");  
slot3.loadFile("{PROJECT_FOLDER}Light Switch Flick 4.wav");  

};

Content.getComponent("Effect").setControlCallback(onEffectControl);


const laf = Engine.createGlobalScriptLookAndFeel();

Engine.loadAudioFilesIntoPool();

laf.registerFunction("drawRotarySlider", function(g, obj)
{
	var a = obj.area;
	var thick = 3;
	var thick2 = 2;
	var thick3 = 6.5;
	
	if (obj.text.indexOf("big") != -1)
	{
		g.setColour(Colours.withAlpha(obj.itemColour1, 0.15));
		g.drawEllipse([4, 4, a[2] - 8, a[3] - 8], thick / 3);
		
		g.setColour(obj.textColour);
		g.fillEllipse([4, 7, a[2] - 8, a[3] - 8]);

		g.setColour(obj.bgColour);
		g.fillEllipse([10, 10, a[2] - 20, a[3] - 20]);
		
		g.setColour(obj.itemColour1);
		g.drawEllipse([8, 8, a[2] - 16, a[3] - 16], 3);
		
	//	g.setGradientFill([0x000000, 0, a[3] / 2, Colours.withAlpha(Colours.black, 0.6), a[3] / 2, a[3]]);
		g.fillEllipse([9, 9, a[2] - 18, a[3] - 18]);
									
		var start = 2;
		var end = start * 2 * obj.valueNormalized - start;
			
		g.rotate(end, [a[2] / 2, a[3] / 2]);
			
		g.setColour(obj.itemColour2);
		g.fillEllipse([a[2] / 2 - thick3 / 2, 12, 8, 8]);
					
	}
	else if (obj.text.indexOf("small") != -1)
	{
		g.setColour(Colours.withAlpha(obj.itemColour1, 0.15));
		g.drawEllipse([4, 4, a[2] - 8, a[3] - 8], thick / 3);
		
		g.setColour(obj.textColour);
		g.fillEllipse([4, 7, a[2] - 8, a[3] - 8]);
		
		g.setColour(obj.bgColour);
		g.fillEllipse([9, 9, a[2] - 18, a[3] - 18]);
		
		g.setColour(obj.itemColour1);
		g.drawEllipse([8, 8, a[2] - 16, a[3] - 16], 3);
		
	//	g.setGradientFill([0x000000, 0, a[3] / 2, Colours.withAlpha(Colours.black, 0.6), a[3] / 2, a[3]]);
		g.fillEllipse([8, 8, a[2] - 16, a[3] - 16]);
									
		var start = 2;
		var end = start * 2 * obj.valueNormalized - start;
			
		g.rotate(end, [a[2] / 2, a[3] / 2]);
			
		g.setColour(obj.itemColour2);
		g.fillEllipse([a[2] / 2 - thick3 / 2, 10, 6.5, 6.5]);
					
	}

});

// Get the script FX module as the display buffer source
const var dp = Synth.getDisplayBufferSource("Script FX1");

// Get the first external display buffer (in our case the peak node that we set 
// to be the External Display Buffer #1)
const var rb = dp.getDisplayBuffer(0);

const var BUFFER_LENGTH = 16384;

// These are the properties that you can give the peak meter
// NumChannels must be 1, but you can set another buffer length for the display
const var properties = {
  "BufferLength": BUFFER_LENGTH,
  "NumChannels": 1
};

// This will override the default properties of the node. You can use the same
// procedure to customize the FFT properties too (like Window type, etc).
rb.setRingBufferProperties(properties);

// Now let's add a simple panel for displaying the signal
const var P1 = Content.getComponent("P1");



// We need to periodically query the ring buffer to create a signal path
// so we use the panel's timers for this.
P1.setTimerCallback(function()
{
	// The autocomplete fails somehow with supplying you the methods,
	// but you can look in the API Browser under ScriptRingBuffer
	// This method creates a path object to mirror the peak display.
	this.data.buffer = rb.createPath(this.getLocalBounds(0), // area
									 [0, BUFFER_LENGTH, 0, 1.0], // [x_start, x_end, y_min, y_max]
									 0.0 // start value (the initial value at position 0)
									 );
	this.repaint();
});

P1.startTimer(30);

P1.setPaintRoutine(function(g)
{
	g.fillAll(0xFF86291F);
	g.setColour(0xFF9C741C);
	g.fillPath(this.data.buffer, this.getLocalBounds(0));
});

const var SettingsPanel = Content.getComponent("SettingsPanel");



inline function onShowSettingsControl(component, value)
{
	SettingsPanel.showControl(value);
};

Content.getComponent("ShowSettings").setControlCallback(onShowSettingsControl);




function onNoteOn()
{
	
}
 function onNoteOff()
{
	
}
 function onController()
{
	
}
 function onTimer()
{
	
}
 function onControl(number, value)
{
	
}
 