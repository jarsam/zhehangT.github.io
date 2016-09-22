---
title: Github+Hexo博客搭建思路
date: 2016-09-22 18:35:42
tags: 
categories: 
- 折腾日记
description: 记录利用Github+Hexo进行博客搭建的思路，以及踩过的坑
---
<!-- more -->

# 摘要
这篇文章主要记录了利用Github+Hexo进行博客搭建的思路，但是略过了很多具体的细节。因此其实这篇文章提供的大多都是为什么，而不是怎么做。以及一些踩过的坑。整个搭建过程基于Ubuntu16.04，搭建时间2016年9月。

# 思路
1.利用Hexo在本地构建博客
2.将构建完成的博客部署到Github上
3.利用域名解析将自己的域名解析到Github上
4.解决百度爬虫无法爬取Github
5.本地博客的备份问题

# Hexo
Hexo是一个简单轻便的博客系统。Hexo可以将Markdown文件自动转换成静态网页，正是基于这个特性，可以利用Github pages的静态页面托管服务，将Hexo博客系统托管到Github上。
对比使用博客园，CSDN这种类型的博客，通过Github+Hexo搭建的博客具有高度的可定制性。Hexo提供了成百上千的主题，也可以学习自己制作主题。而且数据在本地有一份镜像，只要做好备份就可以保证，一切都在掌控之中。
Hexo的使用，官方网站介绍的非常详细。其中最常用的几个命令如下：
```python
hexo new “博客名” #创建新博客，在source/_posts目录下会产生一个md文件，博客就在这里面写
hexo server      #运行该命令后，可以在localhost:4000访问本地博客
hexo clean       #清理本地空间
hexo generation  #生成静态文件
hexo deploy      #根据_config.yml文件中的deploy选项，进行自动部署

```
了解了基本使用之后，可以从github上下载喜欢的hexo主题进行配置和修改，甚至可以自己编写主题。So,做一个有个性的人，尽情的折腾吧!
> ps：我觉得好看的几个主题都被用得烂大街了，不开心。比如NexT。


# Github pages
Github pages可以免费托管静态页面，因此可以将Hexo博客部署到Github pages上。部署成功后可以通过username.github.io访问Hexo博客。全程傻瓜式操作。需要注意的是要使用Github pages服务，创建的项目名必须与用户名一致。并且静态页面要托管在Master分支下。

原则上，只需要将Hexo产生的静态页面文件夹下的文件全部上传到Master分支，就可以完成部署。但为了方便起见，可以利用Hexo的deploy命令完成一键部署。

主要步骤如下：
1.配置可以利用SSH自动登录Github。虽然不是必须的，但是可以免除每次部署都要输入用户名和密码的麻烦。
2.安装hexo-deployer-git插件。命令如下。
```
npm install hexo-deployer-git --save
```
3.编辑站点的配置文件_config.yml 

```
deploy:
  type: git
  repo: <repository url>
  branch: [branch]
```

# 域名
如果觉得username.github.io这样的域名不够高大上，可以通过购买自己喜欢的域名，然后与Github pages进行绑定。
国内的域名购买主要有百度开放云，万网，新网等。完成域名购买之后进入相应网站的管理控制台，进行域名解析。
域名解析方式有两种，A解析将域名直接指向指定的IP。CNAME解析将域名解析到另一个指定的域名。部署在github上的博客的IP可以通过Ping命令得到。即ping username.github.io
> ps.买5年win的域名，花了25块。现在觉得这个域名好傻逼。

# Coding.net
将域名绑定到Github pages上之后，发现百度爬虫无法爬取博客内容。这会导致无法通过百度搜索到刚刚辛辛苦苦搭建的博客。原因是Github禁止了百度爬虫的爬取。为了解决这个问题，选择的方案是将Hexo博客同时部署到Github和Coding.net上。国内线路访问时，将域名解析到Coding.net上，而国外线路访问时，将域名解析到Github上。Hexo的站点配置文件_cofig.yml要类似这么写。
```python
deploy:
- type: git
  repo: <repository url>  #github的ssh地址
  branch: [branch]
- type: git
  repo: <repository url>  #coding.net的ssh地址
  branch: [branch]
```

经过几次实验，这个方案好像是可行的。

# 备份
常在河边走，哪有不湿鞋。Hexo博客的编写都是基于本地的。因此备份是为了防止某一天系统莫名其妙崩了或者硬盘莫名其妙挂掉了。其次如果需要在多台机器上进行博客的撰写，备份也可以作为本地文件的同步方案。理论上只要将这个Hexo博客目录进行备份，就可以在必要时进行恢复或同步。选择的方式是直接将Hexo目录备份到Github项目的另一个分支下。需要注意的是备份要通过.gitignore文件忽略.deploy_git文件夹和public文件夹，因为这两个文件夹存放的是生成的静态网页，这部分内容不需要备份。


# 几个坑
- Hexo是基于node.js的。百度到了很多关于Ubuntu下node.js的安装，有基于deb的，有自己编译的，但不知道为什么全都不管用。所以最好还是根据Hexo官网提供的方法进行安装，三条命令搞定。
- 域名解析时，将域名通过CNAME解析到username.github.io时没有成功。原因未明。通过A解析到IP时成功了。可能是DNS解析服务器的问题。
- 基于上面的问题，在网上看了一些大神分析，购买域名的网站提供的DNS解析服务比较辣鸡，建议转换到dnspod这种第三方的DNS解析服务商。只需要在购买域名的网站控制台里更改DNS服务器地址为dnspod的地址，即可更换DNS解析服务商。更换之后域名的解析需要登录dnspod完成。


















