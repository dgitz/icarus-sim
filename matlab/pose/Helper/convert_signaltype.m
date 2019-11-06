## Copyright (C) 2019 David Gitz
## 
## This program is free software; you can redistribute it and/or modify it
## under the terms of the GNU General Public License as published by
## the Free Software Foundation; either version 3 of the License, or
## (at your option) any later version.
## 
## This program is distributed in the hope that it will be useful,
## but WITHOUT ANY WARRANTY; without even the implied warranty of
## MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
## GNU General Public License for more details.
## 
## You should have received a copy of the GNU General Public License
## along with this program.  If not, see <http://www.gnu.org/licenses/>.

## -*- texinfo -*- 
## @deftypefn {} {@var{retval} =} convert_signaltype (@var{input1}, @var{input2})
##
## @seealso{}
## @end deftypefn

## Author: David Gitz <robot@dgitzdev>
## Created: 2019-11-03

function [conversion_factor,signal_type] = convert_signaltype (units)
global SignalType
if(strcmp(units,'meter/s^2') == 1)
  signal_type = SignalType.SIGNALTYPE_ACCELERATION;
  conversion_factor = 1.0;
elseif(strcmp(units,'deg/s') == 1)
  signal_type = SignalType.SIGNALTYPE_ROTATION_RATE;
  conversion_factor = pi*180.0;
elseif(strcmp(units,'uTesla') == 1)
  signal_type = SignalType.SIGNALTYPE_MAGNETIC_FIELD;
  conversion_factor = 1.0;
else
  disp(['ERROR: Unit Type: ' units ' NOT SUPPORTED.']);
end
endfunction
