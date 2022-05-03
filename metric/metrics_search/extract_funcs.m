% Create monomial functions
w_poly_fcn = mss2fnc(w_poly,x,randn(length(x),2));
dw_poly_fcn = mss2fnc(dw_poly_dx,x,randn(length(x),2));
% write a few functions to m files
W_exec = 'W_eval = @(ml)';
w_eval_str = '';
for i = 1:length(w_poly)
    if i<length(w_poly)
        W_exec = strcat(W_exec,sprintf('W_coef(:,:,%d)*ml(%d) +',i,i));
        if i == 1
            w_eval_str = strcat(w_eval_str,mat2str(W_coef(:,:,i),5),sprintf('*ml(%d) +',i),'...\n');
        else
            w_eval_str = [w_eval_str,'      ',mat2str(W_coef(:,:,i),5),sprintf('*ml(%d) +',i),'...\n'];
        end
    else
        W_exec = strcat(W_exec,sprintf('W_coef(:,:,%d)*ml(%d);',i,i));
        w_eval_str = [w_eval_str,'      ',mat2str(W_coef(:,:,i)),sprintf('*ml(%d);',i)];
    end
end
w_poly_fcn_str = func2str(w_poly_fcn);
dw_poly_fcn_str = func2str(dw_poly_fcn);
w_poly_fcn_str = strcat('ml = ', w_poly_fcn_str(5:end),';\n');
dw_poly_fcn_str = strcat('ml = ', dw_poly_fcn_str(5:end),';\n');


w_fcn_str = strcat('function W = W_fcn1(x)\n', w_poly_fcn_str,'W = ',w_eval_str,'\nend');
dW_dalpha_str = strcat('function dW_dalpha = dW_dalpha_fcn(x)\n', dw_poly_fcn_str,'dW_dalpha = ',w_eval_str,'\nend');

fid = fopen('W_fcn1.m','w');
fprintf(fid,w_fcn_str);
fclose(fid);

fid = fopen('dW_dalpha_fcn.m','w');
fprintf(fid,dW_dalpha_str);
fclose(fid);

% create the function handle
eval(W_exec);
W_fcn = @(x) W_eval(w_poly_fcn(x));

dW_dalpha = @(x) W_eval(dw_poly_alpha(x));

dW_dxi_fcn = @(i,x) (i==2)*dW_dalpha_fcn(x);  % the second state is alpha
dW_dt_fcn = @(x,u) W_eval(dw_poly_dt_fcn([x;u])); 




