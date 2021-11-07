function [] = hist_pdf(N, X, flag)

if flag == 1 % Gaussian distro
    nb = sqrt(N);
    [n,b] = hist(X,nb);
    figure
    bar(b, n/(b(2)-b(1))/sum(n), 1)
    hold on 
    pdfNorm = @(x, m, sigma) 1/sqrt(2*pi)/sigma*exp(-0.5*((x-m)/sigma).^2);
    % mean = 0  sigma = 1
    X = linspace(-3, 3, 100); %vedi estremi hist
    PDF = pdfNorm(X,0,1);
    plot(X, PDF, '*b');
    hold off
    title('Normalized histogram and pdf of Gaussian distribution');
    xlabel('x');
    ylabel('y');
end


if flag == 2 % Uniform distro
    nb = sqrt(N);
    [n,b] = hist(X,nb);
    figure
    bar(b, n/(b(2)-b(1))/sum(n), 1)
    hold on 
    X = linspace(-1.7, 1.7, 100); %vedi estremi hist
    A = -2;
    B = 2;
    pdfUni = unifpdf (X, A, B);
    plot (X, pdfUni)
    PDF = pdfUni; 
    plot(X, PDF, '*b');
  % X = linspace(-3, 3, 100); %vedi estremi hist
    hold off  
    title('Normalized histogram and pdf of Uniform distribution');
    xlabel('x');
    ylabel('y');
end


if flag == 3 % Gaussian mixture distro
    nb = sqrt(N);
    [n,b] = hist(X,nb);
    figure
    bar(b, n/(b(2)-b(1))/sum(n), 1)
    hold on
    pdf1 = makedist ('Normal', 'mu', 0.95, 'sigma', 1-(0.95).^2);
    pdf2 = makedist ('Normal', 'mu', -0.95, 'sigma', 1-(-0.95).^2);
    X = -1.75: 0.01: 1.75;
    pdfMixNorm = (1-(0.95).^2) * pdf (pdf1, X) + (1-(-0.95).^2) * pdf (pdf2, X) ;
    PDF = pdfMixNorm;
    plot(X, PDF, '*b');
    hold off
    title('Normalized histogram and pdf of Gaussian mixture distribution');
    xlabel('x');
    ylabel('y');
end

end
