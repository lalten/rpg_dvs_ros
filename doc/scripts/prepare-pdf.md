#Get PDF of Readme.md

* Compile to html or use Github to display page
* For Github
  - View only the .md file
  - Remove header in web developer toolbox
  - edit css as described below
  - print -> save as PDF
* Merge with PDF of cover page and work description
* Be happy and submit your work



```
//change
.file #readme .markdown-body {
    border: 0;
    padding: 45px;
    border-radius: 0;
    font-size: 150%;
    font-family: "Latin Modern Roman";
    text-align: justify;
}

//add
.markdown-body pre code, .markdown-body pre tt {
    white-space: pre-wrap;
}
```
