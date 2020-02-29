% %% Copyright (C) 2019 david
% 
%  This program is free software; you can redistribute it and/or modify it
%  under the terms of the GNU General Public License as published by
%  the Free Software Foundation; either version 3 of the License, or
%  (at your option) any later version.
%  
%  This program is distributed in the hope that it will be useful,
%  but WITHOUT ANY WARRANTY; without even the implied warranty of
%  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
%  GNU General Public License for more details.
%  
%  You should have received a copy of the GNU General Public License
%  along with this program.  If not, see <http://www.gnu.org/licenses/>.
%  -*- texinfo -*- 
%  @deftypefn {} {@var{retval} =} compute_variance_covariance_matrix (@var{input1}, @var{input2})
% 
%  @seealso{}
%  @end deftypefn
% 
%  Author: david <david@HOME-LAPTOP-WIN>
%  Created: 2019-01-04
% Validation: See https://stattrek.com/matrix-algebra/covariance-matrix.aspx
%   Inputs: z1 = [90 90 60 60 30]; z2 = [60 90 60 60 30]; z3 = [90 30 60 90 30];
%   >>r = compute_variance_covariance_matrix([z1; z2; z3]')
% %  r =
% %
% %   504.00000   360.00000   180.00000
% %   360.00000   360.00000     0.00000
% %   180.00000     0.00000   720.00000
% Usage:
% Input:
%  [X] = Signal Measurement Matrix of size c x N, where N = number of samples and c is number of input signals

function [R] = compute_variance_covariance_matrix (X)
  [a,b] = size(X);
  inputs = b;
  samples = a;
  one = ones(samples,1);
  x = X-(one*one'*X)/samples;
  V = (1/samples)*x'*x;
  R = V;
  
