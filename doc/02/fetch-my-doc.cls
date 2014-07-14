\NeedsTeXFormat{LaTeX2e}
\ProvidesClass{fetch-my-doc}
\LoadClass[a4paper,parskip=half-,toc=listof]{scrartcl}

\RequirePackage[table]{xcolor}

\usepackage[utf8]{inputenc}
\usepackage{lmodern}
\usepackage[ngerman]{babel}
%\usepackage{calc}
%\usepackage{csquotes}
\usepackage{fancyhdr}
\usepackage{booktabs}
\usepackage{tabularx}
%\usepackage[gen]{eurosym}
%\usepackage[norounding,german]{rccol}
\usepackage{graphicx}
\usepackage{float}
%\usepackage{rotating}
%\usepackage{rotfloat}
%\usepackage{lastpage}
%\usepackage[style=alphabetic,natbib=true,maxnames=99]{biblatex}
%\usepackage{bibgerm}
%\usepackage{natbib}
%\usepackage{enumerate}
%\usepackage{keystroke}
\usepackage[leqno]{amsmath}
\usepackage{amssymb}
%\usepackage{listings}
%\lstset{numbers=left, numberstyle=\tiny, numbersep=5pt,tabsize=2,breaklines=true}
%\usepackage{verbatim}
%\usepackage{moreverb}
\usepackage[breaklinks=true,pdfborder={0 0 0}]{hyperref}
\usepackage{pgf}
\usepackage{tikz}
%\usepackage[weather]{ifsym}
%\usepackage{slashbox}
%\usepackage{ipa}	%\rightt
%\usepackage{MnSymbol}
%\usepackage{fancybox}
\usepackage[colorinlistoftodos,backgroundcolor=orange!50,bordercolor=orange]{todonotes}
\usepackage{longtable}
%\usepackage{setspace}
%\usepackage{totcount}
\usepackage[backend=bibtex8,style=reading,sortlocale=de_DE,entrykey=false,useauthor=false]{biblatex}
\renewcommand{\bibliography}[1]{\addbibresource{#1.bib}}
\newcommand{\cit}[1]{[\cite{#1}]}

%\usetikzlibrary{calc,decorations.pathreplacing,patterns,calc,intersections,positioning}
\usetikzlibrary{arrows,automata,decorations.pathmorphing,backgrounds,positioning,fit,petri,through}

\setcounter{secnumdepth}{3}
\let\verbatiminput=\verbatimtabinput

\input{todo.tex}

\newcommand{\autor}[1]{\vspace{-4ex}{\sc \footnotesize Autor(en) dieses Abschnitts: #1.}

}

\newcommand{\libdir}{}

\newcommand{\ks}[1]{\raisebox{1pt}{\keystroke{#1}}}
\newcommand{\RM}[1]{\MakeUppercase{\romannumeral #1}}
\newcommand{\mcc}[1]{\multicolumn{1}{c}{#1}}
\newcommand{\mcr}[1]{\multicolumn{1}{r}{#1}}
\newcommand{\mcl}[1]{\multicolumn{1}{l}{#1}}

\newcommand{\HRule}{\rule{\linewidth}{0.5mm}}

\newcommand{\on}[1]{\operatorname{#1}}
\newcommand{\sepline}{\hspace{-2em}\noindent\rule{.33\textwidth}{0.5pt}\\}
\newcommand{\tabmath}[1]{$#1$\vspace{.5ex}}

\newcommand{\universitaet}{Universität Bremen}
\newcommand{\fach}{Lisp Tutorial}
\newcommand{\gruppe}{0}
\newcommand{\dozent}{Jan Winkler und Moritz Matthias Tenorth}
\newcommand{\tutor}{Jan Winkler}
\newcommand{\semester}{Sommersemester 2014}
\newcommand{\datum}{}
\newcommand{\abgabe}{}
\newcommand{\version}{}
\newcommand{\typ}{}
\newcommand{\titel}{}
\newcommand{\mgEinsVorname}{Andreas}
\newcommand{\mgZweiVorname}{Hannes}
\newcommand{\mgDreiVorname}{Tom}
\newcommand{\mgEinsNachname}{Romero Früh}
\newcommand{\mgZweiNachname}{Welling}
\newcommand{\mgDreiNachname}{Gehrke}
\newcommand{\mgEinsMail}{aromero@uni-bremen.de}
\newcommand{\mgZweiMail}{hwelling@uni-bremen.de}
\newcommand{\mgDreiMail}{tom.gehrke@uni-bremen.de}
\newcommand{\rechte}{\mgEinsNachname, \mgZweiNachname, \mgDreiNachname}

%\addtolength{\footskip}{3ex}
\addtolength{\textheight}{60pt}
\newcommand{\versch}{30pt}
%\addtolength{\textheight}{-\versch}

\fancypagestyle{FirstPage}{
	%\newgeometry{left=4cm}
	\fancyhfoffset{\marginparwidth}
	\addtolength{\headheight}{\versch}
	%\addtolength{\textheight}{\versch}
	\rhead{
		\parbox[t][\headheight]{\textwidth}{\raggedleft%
			{\datum}\linebreak\linebreak\linebreak%
		}%
	}%
	\lhead{
		\parbox[t][\headheight]{\textwidth}{\raggedright%
			{\universitaet}, {\semester}\linebreak%
			{\fach}\ bei {\dozent}\linebreak%
			Tutor: {\tutor}, Gruppe: {\gruppe}
		}%
	}%
}

\fancypagestyle{Landscape}{
	\renewcommand{\headrulewidth}{0pt}
	\renewcommand{\footrulewidth}{0pt}
	\fancyhf{}
}

\pagestyle{fancy}
\fancyhf{}
\fancyhfoffset{\marginparwidth}
\rhead{\leftmark}
\lhead{\fach\ --- \titel}
\rfoot{Seite \pagemark\ von \pageref{LastPage}}
\lfoot{\small{\copyright\ \number\year\ \rechte }}

\endinput
