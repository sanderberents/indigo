// Pegasus Indigo wheel simulator for Arduino
//
// Copyright (c) 2023 CloudMakers, s. r. o.
// All rights reserved.
//
// You can use this software under the terms of 'INDIGO Astronomy
// open-source license' (see LICENSE.md).
//
// THIS SOFTWARE IS PROVIDED BY THE AUTHORS 'AS IS' AND ANY EXPRESS
// OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
// GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
// WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
// NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#ifdef ARDUINO_SAM_DUE
#define Serial SerialUSB
#endif

int current_filter = '1';

void setup() {
  Serial.begin(9600);
  Serial.setTimeout(1000);
  while (!Serial)
    ;
}

void loop() {
  String command = Serial.readStringUntil('\n');
  if (command.equals("W#")) {
    Serial.println("FW_OK");
  } else if (command.equals("WA")) {
    Serial.print("FW_OK:");
    Serial.print(current_filter);
    Serial.println(":0");
  } else if (command.equals("WF")) {
    Serial.print("WF:");
    Serial.println(current_filter);
  } else if (command.equals("WV")) {
    Serial.println("WV:1.0");
  } else if (command.startsWith("WM:")) {
    current_filter = command.substring(3).toInt();
    Serial.println(command);
  } else if (command.equals("WR")) {
    Serial.println("WR:0");
  } else if (command.equals("WI")) {
    Serial.println("WI:1");
  } else if (command.equals("WQ")) {
  }
}
