// mbedRPC Javascript Interface using HTTP
// sford and M Walker
// A javascript interface for talking to mbed rpc over http
//
//Copyright (c) 2010 ARM Ltd
// 
//Permission is hereby granted, free of charge, to any person obtaining a copy
//of this software and associated documentation files (the "Software"), to deal
//in the Software without restriction, including without limitation the rights
//to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
//copies of the Software, and to permit persons to whom the Software is
//furnished to do so, subject to the following conditions:
// 
//The above copyright notice and this permission notice shall be included in
//all copies or substantial portions of the Software.
// 
//THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
//IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
//FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
//AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
//LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
//OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
//THE SOFTWARE.


//Transport Mechanisms   ---------------------------------------------------------------

function post(url) {
    http = (window.XMLHttpRequest) ? new XMLHttpRequest() : new ActiveXObject("Microsoft.XMLHTTP");
    if(!http) return false;
    http.open("GET", url, false);                             
    http.send(null);   
    return http.responseText;
	                                  
}

function mbed(){
	//No action needs to be taken
}

mbed.prototype.rpc = function(object, method, arguments){
    //This should be overridden by the a transport mechanism
}

HTTPRPC.prototype = new mbed();
HTTPRPC.prototype.constructor = HTTPRPC
function HTTPRPC(){
	//unlike other languages this doesn't need to be passed an address as it will only be allowed to communicate with itself
}


HTTPRPC.prototype.rpc= function(object, method, arguments) {
    return post("/rpc/" + object + "/" + method + "," + arguments.join(","));
}

// Pin Names to Allow pins to be reffered to as a type rather than a string or number

function pin(name){
	this._name = name;
}

LED1 = new pin("LED1");
LED2 = new pin("LED2");
LED3 = new pin("LED3");
LED4 = new pin("LED4");

p5 = new pin("p5");
p6 = new pin("p6");
p7 = new pin("p7");
p8 = new pin("p8");
p9 = new pin("p9");
p10 = new pin("p10");
p11 = new pin("p11");
p12 = new pin("p12");
p13 = new pin("p13");
p14 = new pin("p14");
p15 = new pin("p15");
p16 = new pin("p16");
p17 = new pin("p17");
p18 = new pin("p18");
p19 = new pin("p19");
p20 = new pin("p20");
p21 = new pin("p21");
p22 = new pin("p22");
p23 = new pin("p23");
p24 = new pin("p24");
p25 = new pin("p25");
p26 = new pin("p26");
p27 = new pin("p27");
p28 = new pin("p28");
p29 = new pin("p29");
p30 = new pin("p30");


// interface functions   ----------------------------------------------------------------

//*************************** DigitalOut ************************************************

function DigitalOut(this_mbed, mpin) {
	if(typeof mpin != "string"){	
		//create a new object
		this._mbed = this_mbed;
    		this.name = this_mbed.rpc("DigitalOut", "new", [mpin._name]); 
	}else{
		//Tie to an existing object
		this._mbed = this_mbed;
		this.name = mpin;
	}
}

DigitalOut.prototype.write = function(value) {
	this._mbed.rpc(this.name, "write", [value]);
}

DigitalOut.prototype.read = function() {
	return parseInt(this._mbed.rpc(this.name, "read", [""]));
}
//*************************** DigitalIn ************************************************

function DigitalIn(this_mbed, mpin) {
	if(typeof mpin != "string"){	
		//create a new object
		this._mbed = this_mbed;
    		this.name = this._mbed.rpc("DigitalIn", "new", [mpin._name]); 
	}else{
		//Tie to an existing object
		this._mbed = this_mbed;
		this.name = mpin;
	}

}

DigitalIn.prototype.read = function() {
	return parseInt(this._mbed.rpc(this.name, "read", [""]));
}

//*************************** AnalogOut ************************************************

function AnalogOut(this_mbed, mpin) {
	if(typeof mpin != "string"){	
		//create a new object
		this._mbed = this_mbed;
    		this.name = this._mbed.rpc("AnalogOut", "new", [mpin._name]); 
	}else{
		//Tie to an existing object
		this._mbed = this_mbed;
		this.name = mpin;
	}
}

AnalogOut.prototype.write = function(value) {
	this._mbed.rpc(this.name, "write", [value]);
}

AnalogOut.prototype.write_u16 = function(value) {
	this._mbed.rpc(this.name, "write_u16", [value]);
}

AnalogOut.prototype.read = function() {
	return parseFloat(this._mbed.rpc(this.name, "read", [""]));
}

//*************************** AnalogIn ************************************************

function AnalogIn(this_mbed, mpin) {
	if(typeof mpin != "string"){	
		//create a new object
		this._mbed = this_mbed;
    		this.name = this._mbed.rpc("AnalogIn", "new", [mpin._name]); 
	}else{
		//Tie to an existing object
		this._mbed = this_mbed;
		this.name = mpin;
	}
}

AnalogIn.prototype.read = function() {
	return parseFloat(this._mbed.rpc(this.name, "read", [""]));
}

AnalogIn.prototype.read_u16 = function() {
	return parseInt(this._mbed.rpc(this.name, "read_u16", [""]));
}

//*************************** PwmOut ************************************************

function PwmOut(this_mbed, mpin) {
	if(typeof mpin != "string"){	
		//create a new object
		this._mbed = this_mbed;
    		this.name = this._mbed.rpc("PwmOut", "new", [mpin._name]); 
	}else{
		//Tie to an existing object
		this._mbed = this_mbed;
		this.name = mpin;
	}
}

PwmOut.prototype.write = function(value) {
	this._mbed.rpc(this.name, "write", [value]);
}

PwmOut.prototype.read = function() {
    return parseFloat(this._mbed.rpc(this.name, "read", [""]));
}

PwmOut.prototype.period = function(value) {
	this._mbed.rpc(this.name, "period", [value]);
}

PwmOut.prototype.period_ms = function(value) {
	this._mbed.rpc(this.name, "period_ms", [value]);
}

PwmOut.prototype.period_us = function(value) {
	this._mbed.rpc(this.name, "period_us", [value]);
}

PwmOut.prototype.pulsewidth = function(value) {
	this._mbed.rpc(this.name, "pulsewidth", [value]);
}

PwmOut.prototype.pulsewidth_ms = function(value) {
	this._mbed.rpc(this.name, "pulsewidth_ms", [value]);
}

PwmOut.prototype.pulsewidth_us = function(value) {
	this._mbed.rpc(this.name, "pulsewidth_us", [value]);
}

//*************************** serial ************************************************

function Serial(this_mbed, tx, rx) {
	if(typeof tx != "string"){	
		//create a new object
		this._mbed = this_mbed;
    		this.name = this._mbed.rpc("Serial", "new", [tx._name, rx._name]); 
	}else{
		//Tie to an existing object
		this._mbed = this_mbed;
		this.name = tx;
	}

}

Serial.prototype.putc = function(value) {
	this._mbed.rpc(this.name, "putc", [value]);
}

Serial.prototype.getc = function(value) {
	return parseInt(this._mbed.rpc(this.name, "getc", [""]));
}

Serial.prototype.readable = function(value) {
	return parseInt(this._mbed.rpc(this.name, "readable", [""]));
}

Serial.prototype.writeable = function(value) {
	return parseInt(this._mbed.rpc(this.name, "writeable", [""]));
}


//*************************** RPCVariable ************************************************

function RPCVariable(this_mbed, name) {
	this._mbed = this_mbed;
        this.name = name; 
}

RPCVariable.prototype.write = function(value) {
	this._mbed.rpc(this.name, "write", [value]);
}

RPCVariable.prototype.read_int = function() {
	return parseInt(this._mbed.rpc(this.name, "read", [""]));
}

RPCVariable.prototype.read_float = function() {
	return parseFloat(this._mbed.rpc(this.name, "read", [""]));
}

RPCVariable.prototype.read = function() {
	return (this._mbed.rpc(this.name, "read", [""]));
}

//*************************** RPCFunction ************************************************

function RPCFunction(this_mbed, name) {
	this._mbed = this_mbed;
        this.name = name; 
}

RPCVariable.prototype.run = function(value) {
	return (this._mbed.rpc(this.name, "run", [value]));
}