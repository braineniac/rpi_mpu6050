





<!DOCTYPE html>
<html lang="en">
  <head>
    <meta charset="utf-8">
  <link rel="dns-prefetch" href="https://github.githubassets.com">
  <link rel="dns-prefetch" href="https://avatars0.githubusercontent.com">
  <link rel="dns-prefetch" href="https://avatars1.githubusercontent.com">
  <link rel="dns-prefetch" href="https://avatars2.githubusercontent.com">
  <link rel="dns-prefetch" href="https://avatars3.githubusercontent.com">
  <link rel="dns-prefetch" href="https://github-cloud.s3.amazonaws.com">
  <link rel="dns-prefetch" href="https://user-images.githubusercontent.com/">



  <link crossorigin="anonymous" media="all" integrity="sha512-RPWwIpqyjxv5EpuWKUKyeZeWz9QEzIbAWTiYOuxGieUq7+AMiZbsLeQMfEdyEIUoNjLagHK0BEm92BmXnvaH4Q==" rel="stylesheet" href="https://github.githubassets.com/assets/frameworks-40c1c9d8ff06284fb441108e6559f019.css" />
  <link crossorigin="anonymous" media="all" integrity="sha512-SUcCfd/JEajhY9PBJRkLqJayKdKJo8njHXOMeNGmff0lsDCXCPwTSiR7u7F/szVtOpfkCQ8hfEvPy6DTeEpPKw==" rel="stylesheet" href="https://github.githubassets.com/assets/site-ee3375059d76a2986813c5b7bd605ab3.css" />
    <link crossorigin="anonymous" media="all" integrity="sha512-3CnDMoFJPvbM39ryV5wc51yRo/6j6eQPt5SOlYaoBZhR9rVL/UZH3ME+wt72nsTlNFaSQ3nXT/0F4sxE1zbA6g==" rel="stylesheet" href="https://github.githubassets.com/assets/github-38162889e1878fa3b887aa360e70ab6c.css" />
    
    
    
    

  <meta name="viewport" content="width=device-width">
  
  <title>PiBits/MPU6050_6Axis_MotionApps20.h at master · richardghirst/PiBits · GitHub</title>
    <meta name="description" content="Contribute to richardghirst/PiBits development by creating an account on GitHub.">
    <link rel="search" type="application/opensearchdescription+xml" href="/opensearch.xml" title="GitHub">
  <link rel="fluid-icon" href="https://github.com/fluidicon.png" title="GitHub">
  <meta property="fb:app_id" content="1401488693436528">

    
    <meta property="og:image" content="https://avatars2.githubusercontent.com/u/1824945?s=400&amp;v=4" /><meta property="og:site_name" content="GitHub" /><meta property="og:type" content="object" /><meta property="og:title" content="richardghirst/PiBits" /><meta property="og:url" content="https://github.com/richardghirst/PiBits" /><meta property="og:description" content="Contribute to richardghirst/PiBits development by creating an account on GitHub." />

  <link rel="assets" href="https://github.githubassets.com/">
  
  <meta name="pjax-timeout" content="1000">
  
  <meta name="request-id" content="1F7A:77D4:D1C637:189E25A:5C70566C" data-pjax-transient>


  

  <meta name="selected-link" value="repo_source" data-pjax-transient>

      <meta name="google-site-verification" content="KT5gs8h0wvaagLKAVWq8bbeNwnZZK1r1XQysX3xurLU">
    <meta name="google-site-verification" content="ZzhVyEFwb7w3e0-uOTltm8Jsck2F5StVihD0exw2fsA">
    <meta name="google-site-verification" content="GXs5KoUUkNCoaAZn7wPN-t01Pywp9M3sEjnt_3_ZWPc">

  <meta name="octolytics-host" content="collector.githubapp.com" /><meta name="octolytics-app-id" content="github" /><meta name="octolytics-event-url" content="https://collector.githubapp.com/github-external/browser_event" /><meta name="octolytics-dimension-request_id" content="1F7A:77D4:D1C637:189E25A:5C70566C" /><meta name="octolytics-dimension-region_edge" content="iad" /><meta name="octolytics-dimension-region_render" content="iad" />
<meta name="analytics-location" content="/&lt;user-name&gt;/&lt;repo-name&gt;/blob/show" data-pjax-transient="true" />



    <meta name="google-analytics" content="UA-3769691-2">


<meta class="js-ga-set" name="dimension1" content="Logged Out">



  

      <meta name="hostname" content="github.com">
    <meta name="user-login" content="">

      <meta name="expected-hostname" content="github.com">
    <meta name="js-proxy-site-detection-payload" content="YTY2NmZkYzIwYTk5OGZkMzdkZjY1OGY3OTcxZjU1MzkwZDJhYzYxZDJhMTFkOWMxOGU5MmZlYTBkYjQ0N2ZlNnx7InJlbW90ZV9hZGRyZXNzIjoiODAuMTEwLjExOC43NiIsInJlcXVlc3RfaWQiOiIxRjdBOjc3RDQ6RDFDNjM3OjE4OUUyNUE6NUM3MDU2NkMiLCJ0aW1lc3RhbXAiOjE1NTA4NjYwMjksImhvc3QiOiJnaXRodWIuY29tIn0=">

    <meta name="enabled-features" content="UNIVERSE_BANNER,MARKETPLACE_PLAN_RESTRICTION_EDITOR,MARKETPLACE_BROWSING_V2">

  <meta name="html-safe-nonce" content="57d16e211ae6a473f82d30c9105103227a097dc9">

  <meta http-equiv="x-pjax-version" content="fe602614af4c1a740e12e3bc8fce8de2">
  

      <link href="https://github.com/richardghirst/PiBits/commits/master.atom" rel="alternate" title="Recent Commits to PiBits:master" type="application/atom+xml">

  <meta name="go-import" content="github.com/richardghirst/PiBits git https://github.com/richardghirst/PiBits.git">

  <meta name="octolytics-dimension-user_id" content="1824945" /><meta name="octolytics-dimension-user_login" content="richardghirst" /><meta name="octolytics-dimension-repository_id" content="5471662" /><meta name="octolytics-dimension-repository_nwo" content="richardghirst/PiBits" /><meta name="octolytics-dimension-repository_public" content="true" /><meta name="octolytics-dimension-repository_is_fork" content="false" /><meta name="octolytics-dimension-repository_network_root_id" content="5471662" /><meta name="octolytics-dimension-repository_network_root_nwo" content="richardghirst/PiBits" /><meta name="octolytics-dimension-repository_explore_github_marketplace_ci_cta_shown" content="false" />


    <link rel="canonical" href="https://github.com/richardghirst/PiBits/blob/master/MPU6050-Pi-Demo/MPU6050_6Axis_MotionApps20.h" data-pjax-transient>


  <meta name="browser-stats-url" content="https://api.github.com/_private/browser/stats">

  <meta name="browser-errors-url" content="https://api.github.com/_private/browser/errors">

  <link rel="mask-icon" href="https://github.githubassets.com/pinned-octocat.svg" color="#000000">
  <link rel="icon" type="image/x-icon" class="js-site-favicon" href="https://github.githubassets.com/favicon.ico">

<meta name="theme-color" content="#1e2327">




  <link rel="manifest" href="/manifest.json" crossOrigin="use-credentials">

  </head>

  <body class="logged-out env-production page-blob">
    

  <div class="position-relative js-header-wrapper ">
    <a href="#start-of-content" tabindex="1" class="px-2 py-4 bg-blue text-white show-on-focus js-skip-to-content">Skip to content</a>
    <div id="js-pjax-loader-bar" class="pjax-loader-bar"><div class="progress"></div></div>

    
    
    


        
<header class="Header header-logged-out  position-relative f4 py-3" role="banner">
  <div class="container-lg d-flex px-3">
    <div class="d-flex flex-justify-between flex-items-center">
        <a class="mr-4" href="https://github.com/" aria-label="Homepage" data-ga-click="(Logged out) Header, go to homepage, icon:logo-wordmark">
          <svg height="32" class="octicon octicon-mark-github text-white" viewBox="0 0 16 16" version="1.1" width="32" aria-hidden="true"><path fill-rule="evenodd" d="M8 0C3.58 0 0 3.58 0 8c0 3.54 2.29 6.53 5.47 7.59.4.07.55-.17.55-.38 0-.19-.01-.82-.01-1.49-2.01.37-2.53-.49-2.69-.94-.09-.23-.48-.94-.82-1.13-.28-.15-.68-.52-.01-.53.63-.01 1.08.58 1.23.82.72 1.21 1.87.87 2.33.66.07-.52.28-.87.51-1.07-1.78-.2-3.64-.89-3.64-3.95 0-.87.31-1.59.82-2.15-.08-.2-.36-1.02.08-2.12 0 0 .67-.21 2.2.82.64-.18 1.32-.27 2-.27.68 0 1.36.09 2 .27 1.53-1.04 2.2-.82 2.2-.82.44 1.1.16 1.92.08 2.12.51.56.82 1.27.82 2.15 0 3.07-1.87 3.75-3.65 3.95.29.25.54.73.54 1.48 0 1.07-.01 1.93-.01 2.2 0 .21.15.46.55.38A8.013 8.013 0 0 0 16 8c0-4.42-3.58-8-8-8z"/></svg>
        </a>
    </div>

    <div class="HeaderMenu HeaderMenu--logged-out d-flex flex-justify-between flex-items-center flex-auto">
      <div class="d-none">
        <button class="btn-link js-details-target" type="button" aria-label="Toggle navigation" aria-expanded="false">
          <svg height="24" class="octicon octicon-x text-gray" viewBox="0 0 12 16" version="1.1" width="18" aria-hidden="true"><path fill-rule="evenodd" d="M7.48 8l3.75 3.75-1.48 1.48L6 9.48l-3.75 3.75-1.48-1.48L4.52 8 .77 4.25l1.48-1.48L6 6.52l3.75-3.75 1.48 1.48L7.48 8z"/></svg>
        </button>
      </div>

        <nav class="mt-0" aria-label="Global">
          <ul class="d-flex list-style-none">
              <li class=" mr-3 mr-lg-3 edge-item-fix position-relative flex-wrap flex-justify-between d-flex flex-items-center ">
                <details class="HeaderMenu-details details-overlay details-reset width-full">
                  <summary class="HeaderMenu-summary HeaderMenu-link px-0 py-3 border-0 no-wrap  d-inline-block">
                    Why GitHub?
                    <svg x="0px" y="0px" viewBox="0 0 14 8" xml:space="preserve" fill="none" class="icon-chevon-down-mktg position-relative">
                      <path d="M1,1l6.2,6L13,1"></path>
                    </svg>
                  </summary>
                  <div class="dropdown-menu flex-auto rounded-1 bg-white px-0 mt-0  p-4 left-n4 position-absolute">
                    <a href="/features" class="py-2 lh-condensed-ultra d-block link-gray-dark no-underline h5 Bump-link--hover" data-ga-click="(Logged out) Header, go to Features">Features <span class="Bump-link-symbol float-right text-normal text-gray-light">&rarr;</span></a>
                    <ul class="list-style-none f5 pb-3">
                      <li class="edge-item-fix"><a href="/features/code-review/" class="py-2 lh-condensed-ultra d-block link-gray no-underline f5" data-ga-click="(Logged out) Header, go to Code review">Code review</a></li>
                      <li class="edge-item-fix"><a href="/features/project-management/" class="py-2 lh-condensed-ultra d-block link-gray no-underline f5" data-ga-click="(Logged out) Header, go to Project management">Project management</a></li>
                      <li class="edge-item-fix"><a href="/features/integrations" class="py-2 lh-condensed-ultra d-block link-gray no-underline f5" data-ga-click="(Logged out) Header, go to Integrations">Integrations</a></li>
                      <li class="edge-item-fix"><a href="/features/actions" class="py-2 lh-condensed-ultra d-block link-gray no-underline f5" data-ga-click="(Logged out) Header, go to Actions">Actions</a>
                      <li class="edge-item-fix"><a href="/features#team-management" class="py-2 lh-condensed-ultra d-block link-gray no-underline f5" data-ga-click="(Logged out) Header, go to Team management">Team management</a></li>
                      <li class="edge-item-fix"><a href="/features#social-coding" class="py-2 lh-condensed-ultra d-block link-gray no-underline f5" data-ga-click="(Logged out) Header, go to Social coding">Social coding</a></li>
                      <li class="edge-item-fix"><a href="/features#documentation" class="py-2 lh-condensed-ultra d-block link-gray no-underline f5" data-ga-click="(Logged out) Header, go to Documentation">Documentation</a></li>
                      <li class="edge-item-fix"><a href="/features#code-hosting" class="py-2 lh-condensed-ultra d-block link-gray no-underline f5" data-ga-click="(Logged out) Header, go to Code hosting">Code hosting</a></li>
                    </ul>

                    <ul class="list-style-none mb-0 border-lg-top pt-lg-3">
                      <li class="edge-item-fix"><a href="/case-studies" class="py-2 lh-condensed-ultra d-block no-underline link-gray-dark no-underline h5 Bump-link--hover" data-ga-click="(Logged out) Header, go to Case studies">Case Studies <span class="Bump-link-symbol float-right text-normal text-gray-light">&rarr;</span></a></li>
                      <li class="edge-item-fix"><a href="/security" class="py-2 lh-condensed-ultra d-block no-underline link-gray-dark no-underline h5 Bump-link--hover" data-ga-click="(Logged out) Header, go to Security">Security <span class="Bump-link-symbol float-right text-normal text-gray-light">&rarr;</span></a></li>
                    </ul>
                  </div>
                </details>
              </li>
              <li class=" mr-3 mr-lg-3">
                <a href="/enterprise" class="HeaderMenu-link no-underline py-3 d-block d-lg-inline-block" data-ga-click="(Logged out) Header, click, go to Enterprise">Enterprise</a>
              </li>

              <li class=" mr-3 mr-lg-3 edge-item-fix position-relative flex-wrap flex-justify-between d-flex flex-items-center ">
                <details class="HeaderMenu-details details-overlay details-reset width-full">
                  <summary class="HeaderMenu-summary HeaderMenu-link px-0 py-3 border-0 no-wrap  d-inline-block">
                    Explore
                    <svg x="0px" y="0px" viewBox="0 0 14 8" xml:space="preserve" fill="none" class="icon-chevon-down-mktg position-relative">
                      <path d="M1,1l6.2,6L13,1"></path>
                    </svg>
                  </summary>

                  <div class="dropdown-menu flex-auto rounded-1 bg-white px-0 pt-2 pb-0 mt-0  p-4 left-n4 position-absolute">
                    <ul class="list-style-none mb-3">
                      <li class="edge-item-fix"><a href="/explore" class="py-2 lh-condensed-ultra d-block link-gray-dark no-underline h5 Bump-link--hover" data-ga-click="(Logged out) Header, go to Features">Explore GitHub <span class="Bump-link-symbol float-right text-normal text-gray-light">&rarr;</span></a></li>
                    </ul>

                    <h4 class="text-gray-light text-normal text-mono f5 mb-2  border-top pt-3">Learn &amp; contribute</h4>
                    <ul class="list-style-none mb-3">
                      <li class="edge-item-fix"><a href="/topics" class="py-2 lh-condensed-ultra d-block link-gray no-underline f5" data-ga-click="(Logged out) Header, go to Topics">Topics</a></li>
                      <li class="edge-item-fix"><a href="/collections" class="py-2 lh-condensed-ultra d-block link-gray no-underline f5" data-ga-click="(Logged out) Header, go to Collections">Collections</a></li>
                      <li class="edge-item-fix"><a href="/trending" class="py-2 lh-condensed-ultra d-block link-gray no-underline f5" data-ga-click="(Logged out) Header, go to Trending">Trending</a></li>
                      <li class="edge-item-fix"><a href="https://lab.github.com/" class="py-2 lh-condensed-ultra d-block link-gray no-underline f5" data-ga-click="(Logged out) Header, go to Learning lab">Learning Lab</a></li>
                      <li class="edge-item-fix"><a href="https://opensource.guide" class="py-2 lh-condensed-ultra d-block link-gray no-underline f5" data-ga-click="(Logged out) Header, go to Open source guides">Open source guides</a></li>
                    </ul>

                    <h4 class="text-gray-light text-normal text-mono f5 mb-2  border-top pt-3">Connect with others</h4>
                    <ul class="list-style-none mb-0">
                      <li class="edge-item-fix"><a href="/events" class="py-2 lh-condensed-ultra d-block link-gray no-underline f5" data-ga-click="(Logged out) Header, go to Events">Events</a></li>
                      <li class="edge-item-fix"><a href="https://github.community" class="py-2 lh-condensed-ultra d-block link-gray no-underline f5" data-ga-click="(Logged out) Header, go to Community forum">Community forum</a></li>
                      <li class="edge-item-fix"><a href="https://education.github.com" class="py-2 pb-0 lh-condensed-ultra d-block link-gray no-underline f5" data-ga-click="(Logged out) Header, go to GitHub Education">GitHub Education</a></li>
                    </ul>
                  </div>
                </details>
              </li>

              <li class=" mr-3 mr-lg-3">
                <a href="/marketplace" class="HeaderMenu-link no-underline py-3 d-block d-lg-inline-block" data-ga-click="(Logged out) Header, go to Marketplace">Marketplace</a>
              </li>

              <li class=" mr-3 mr-lg-3 edge-item-fix position-relative flex-wrap flex-justify-between d-flex flex-items-center ">
                <details class="HeaderMenu-details details-overlay details-reset width-full">
                  <summary class="HeaderMenu-summary HeaderMenu-link px-0 py-3 border-0 no-wrap  d-inline-block">
                    Pricing
                    <svg x="0px" y="0px" viewBox="0 0 14 8" xml:space="preserve" fill="none" class="icon-chevon-down-mktg position-relative">
                       <path d="M1,1l6.2,6L13,1"></path>
                    </svg>
                  </summary>

                  <div class="dropdown-menu flex-auto rounded-1 bg-white px-0 pt-2 pb-4 mt-0  p-4 left-n4 position-absolute">
                    <a href="/pricing" class="pb-2 lh-condensed-ultra d-block link-gray-dark no-underline h5 Bump-link--hover" data-ga-click="(Logged out) Header, go to Pricing">Plans <span class="Bump-link-symbol float-right text-normal text-gray-light">&rarr;</span></a>

                    <ul class="list-style-none mb-3">
                      <li class="edge-item-fix"><a href="/pricing#feature-comparison" class="py-2 lh-condensed-ultra d-block link-gray no-underline f5" data-ga-click="(Logged out) Header, go to Compare features">Compare plans</a></li>
                      <li class="edge-item-fix"><a href="https://enterprise.github.com/contact" class="py-2 lh-condensed-ultra d-block link-gray no-underline f5" data-ga-click="(Logged out) Header, go to Compare features">Contact Sales</a></li>
                    </ul>

                    <ul class="list-style-none mb-0  border-top pt-3">
                      <li class="edge-item-fix"><a href="/nonprofit" class="py-2 lh-condensed-ultra d-block no-underline link-gray-dark no-underline h5 Bump-link--hover" data-ga-click="(Logged out) Header, go to Nonprofits">Nonprofit <span class="Bump-link-symbol float-right text-normal text-gray-light">&rarr;</span></a></li>
                      <li class="edge-item-fix"><a href="https://education.github.com" class="py-2 pb-0 lh-condensed-ultra d-block no-underline link-gray-dark no-underline h5 Bump-link--hover"  data-ga-click="(Logged out) Header, go to Education">Education <span class="Bump-link-symbol float-right text-normal text-gray-light">&rarr;</span></a></li>
                    </ul>
                  </div>
                </details>
              </li>
          </ul>
        </nav>

      <div class="d-flex flex-items-center px-0 text-center text-left">
          <div class="d-lg-flex mr-3">
            <div class="header-search scoped-search site-scoped-search js-site-search position-relative js-jump-to"
  role="combobox"
  aria-owns="jump-to-results"
  aria-label="Search or jump to"
  aria-haspopup="listbox"
  aria-expanded="false"
>
  <div class="position-relative">
    <!-- '"` --><!-- </textarea></xmp> --></option></form><form class="js-site-search-form" data-scope-type="Repository" data-scope-id="5471662" data-scoped-search-url="/richardghirst/PiBits/search" data-unscoped-search-url="/search" action="/richardghirst/PiBits/search" accept-charset="UTF-8" method="get"><input name="utf8" type="hidden" value="&#x2713;" />
      <label class="form-control header-search-wrapper header-search-wrapper-jump-to position-relative d-flex flex-justify-between flex-items-center js-chromeless-input-container">
        <input type="text"
          class="form-control header-search-input jump-to-field js-jump-to-field js-site-search-focus js-site-search-field is-clearable"
          data-hotkey="s,/"
          name="q"
          value=""
          placeholder="Search"
          data-unscoped-placeholder="Search GitHub"
          data-scoped-placeholder="Search"
          autocapitalize="off"
          aria-autocomplete="list"
          aria-controls="jump-to-results"
          aria-label="Search"
          data-jump-to-suggestions-path="/_graphql/GetSuggestedNavigationDestinations#csrf-token=tQseWuELzbnHiZX1UtVREyW104+roPBpSeBW/ep511rVdMmv5jhLqRQhylDYPPAU7SNwHGHfbT/MLeNgp4Goew=="
          spellcheck="false"
          autocomplete="off"
          >
          <input type="hidden" class="js-site-search-type-field" name="type" >
            <img src="https://github.githubassets.com/images/search-key-slash.svg" alt="" class="mr-2 header-search-key-slash">

            <div class="Box position-absolute overflow-hidden d-none jump-to-suggestions js-jump-to-suggestions-container">
              
<ul class="d-none js-jump-to-suggestions-template-container">
  

<li class="d-flex flex-justify-start flex-items-center p-0 f5 navigation-item js-navigation-item js-jump-to-suggestion" role="option">
  <a tabindex="-1" class="no-underline d-flex flex-auto flex-items-center jump-to-suggestions-path js-jump-to-suggestion-path js-navigation-open p-2" href="">
    <div class="jump-to-octicon js-jump-to-octicon flex-shrink-0 mr-2 text-center d-none">
      <svg height="16" width="16" class="octicon octicon-repo flex-shrink-0 js-jump-to-octicon-repo d-none" title="Repository" aria-label="Repository" viewBox="0 0 12 16" version="1.1" role="img"><path fill-rule="evenodd" d="M4 9H3V8h1v1zm0-3H3v1h1V6zm0-2H3v1h1V4zm0-2H3v1h1V2zm8-1v12c0 .55-.45 1-1 1H6v2l-1.5-1.5L3 16v-2H1c-.55 0-1-.45-1-1V1c0-.55.45-1 1-1h10c.55 0 1 .45 1 1zm-1 10H1v2h2v-1h3v1h5v-2zm0-10H2v9h9V1z"/></svg>
      <svg height="16" width="16" class="octicon octicon-project flex-shrink-0 js-jump-to-octicon-project d-none" title="Project" aria-label="Project" viewBox="0 0 15 16" version="1.1" role="img"><path fill-rule="evenodd" d="M10 12h3V2h-3v10zm-4-2h3V2H6v8zm-4 4h3V2H2v12zm-1 1h13V1H1v14zM14 0H1a1 1 0 0 0-1 1v14a1 1 0 0 0 1 1h13a1 1 0 0 0 1-1V1a1 1 0 0 0-1-1z"/></svg>
      <svg height="16" width="16" class="octicon octicon-search flex-shrink-0 js-jump-to-octicon-search d-none" title="Search" aria-label="Search" viewBox="0 0 16 16" version="1.1" role="img"><path fill-rule="evenodd" d="M15.7 13.3l-3.81-3.83A5.93 5.93 0 0 0 13 6c0-3.31-2.69-6-6-6S1 2.69 1 6s2.69 6 6 6c1.3 0 2.48-.41 3.47-1.11l3.83 3.81c.19.2.45.3.7.3.25 0 .52-.09.7-.3a.996.996 0 0 0 0-1.41v.01zM7 10.7c-2.59 0-4.7-2.11-4.7-4.7 0-2.59 2.11-4.7 4.7-4.7 2.59 0 4.7 2.11 4.7 4.7 0 2.59-2.11 4.7-4.7 4.7z"/></svg>
    </div>

    <img class="avatar mr-2 flex-shrink-0 js-jump-to-suggestion-avatar d-none" alt="" aria-label="Team" src="" width="28" height="28">

    <div class="jump-to-suggestion-name js-jump-to-suggestion-name flex-auto overflow-hidden text-left no-wrap css-truncate css-truncate-target">
    </div>

    <div class="border rounded-1 flex-shrink-0 bg-gray px-1 text-gray-light ml-1 f6 d-none js-jump-to-badge-search">
      <span class="js-jump-to-badge-search-text-default d-none" aria-label="in this repository">
        In this repository
      </span>
      <span class="js-jump-to-badge-search-text-global d-none" aria-label="in all of GitHub">
        All GitHub
      </span>
      <span aria-hidden="true" class="d-inline-block ml-1 v-align-middle">↵</span>
    </div>

    <div aria-hidden="true" class="border rounded-1 flex-shrink-0 bg-gray px-1 text-gray-light ml-1 f6 d-none d-on-nav-focus js-jump-to-badge-jump">
      Jump to
      <span class="d-inline-block ml-1 v-align-middle">↵</span>
    </div>
  </a>
</li>

</ul>

<ul class="d-none js-jump-to-no-results-template-container">
  <li class="d-flex flex-justify-center flex-items-center f5 d-none js-jump-to-suggestion p-2">
    <span class="text-gray">No suggested jump to results</span>
  </li>
</ul>

<ul id="jump-to-results" role="listbox" class="p-0 m-0 js-navigation-container jump-to-suggestions-results-container js-jump-to-suggestions-results-container">
  

<li class="d-flex flex-justify-start flex-items-center p-0 f5 navigation-item js-navigation-item js-jump-to-scoped-search d-none" role="option">
  <a tabindex="-1" class="no-underline d-flex flex-auto flex-items-center jump-to-suggestions-path js-jump-to-suggestion-path js-navigation-open p-2" href="">
    <div class="jump-to-octicon js-jump-to-octicon flex-shrink-0 mr-2 text-center d-none">
      <svg height="16" width="16" class="octicon octicon-repo flex-shrink-0 js-jump-to-octicon-repo d-none" title="Repository" aria-label="Repository" viewBox="0 0 12 16" version="1.1" role="img"><path fill-rule="evenodd" d="M4 9H3V8h1v1zm0-3H3v1h1V6zm0-2H3v1h1V4zm0-2H3v1h1V2zm8-1v12c0 .55-.45 1-1 1H6v2l-1.5-1.5L3 16v-2H1c-.55 0-1-.45-1-1V1c0-.55.45-1 1-1h10c.55 0 1 .45 1 1zm-1 10H1v2h2v-1h3v1h5v-2zm0-10H2v9h9V1z"/></svg>
      <svg height="16" width="16" class="octicon octicon-project flex-shrink-0 js-jump-to-octicon-project d-none" title="Project" aria-label="Project" viewBox="0 0 15 16" version="1.1" role="img"><path fill-rule="evenodd" d="M10 12h3V2h-3v10zm-4-2h3V2H6v8zm-4 4h3V2H2v12zm-1 1h13V1H1v14zM14 0H1a1 1 0 0 0-1 1v14a1 1 0 0 0 1 1h13a1 1 0 0 0 1-1V1a1 1 0 0 0-1-1z"/></svg>
      <svg height="16" width="16" class="octicon octicon-search flex-shrink-0 js-jump-to-octicon-search d-none" title="Search" aria-label="Search" viewBox="0 0 16 16" version="1.1" role="img"><path fill-rule="evenodd" d="M15.7 13.3l-3.81-3.83A5.93 5.93 0 0 0 13 6c0-3.31-2.69-6-6-6S1 2.69 1 6s2.69 6 6 6c1.3 0 2.48-.41 3.47-1.11l3.83 3.81c.19.2.45.3.7.3.25 0 .52-.09.7-.3a.996.996 0 0 0 0-1.41v.01zM7 10.7c-2.59 0-4.7-2.11-4.7-4.7 0-2.59 2.11-4.7 4.7-4.7 2.59 0 4.7 2.11 4.7 4.7 0 2.59-2.11 4.7-4.7 4.7z"/></svg>
    </div>

    <img class="avatar mr-2 flex-shrink-0 js-jump-to-suggestion-avatar d-none" alt="" aria-label="Team" src="" width="28" height="28">

    <div class="jump-to-suggestion-name js-jump-to-suggestion-name flex-auto overflow-hidden text-left no-wrap css-truncate css-truncate-target">
    </div>

    <div class="border rounded-1 flex-shrink-0 bg-gray px-1 text-gray-light ml-1 f6 d-none js-jump-to-badge-search">
      <span class="js-jump-to-badge-search-text-default d-none" aria-label="in this repository">
        In this repository
      </span>
      <span class="js-jump-to-badge-search-text-global d-none" aria-label="in all of GitHub">
        All GitHub
      </span>
      <span aria-hidden="true" class="d-inline-block ml-1 v-align-middle">↵</span>
    </div>

    <div aria-hidden="true" class="border rounded-1 flex-shrink-0 bg-gray px-1 text-gray-light ml-1 f6 d-none d-on-nav-focus js-jump-to-badge-jump">
      Jump to
      <span class="d-inline-block ml-1 v-align-middle">↵</span>
    </div>
  </a>
</li>

  

<li class="d-flex flex-justify-start flex-items-center p-0 f5 navigation-item js-navigation-item js-jump-to-global-search d-none" role="option">
  <a tabindex="-1" class="no-underline d-flex flex-auto flex-items-center jump-to-suggestions-path js-jump-to-suggestion-path js-navigation-open p-2" href="">
    <div class="jump-to-octicon js-jump-to-octicon flex-shrink-0 mr-2 text-center d-none">
      <svg height="16" width="16" class="octicon octicon-repo flex-shrink-0 js-jump-to-octicon-repo d-none" title="Repository" aria-label="Repository" viewBox="0 0 12 16" version="1.1" role="img"><path fill-rule="evenodd" d="M4 9H3V8h1v1zm0-3H3v1h1V6zm0-2H3v1h1V4zm0-2H3v1h1V2zm8-1v12c0 .55-.45 1-1 1H6v2l-1.5-1.5L3 16v-2H1c-.55 0-1-.45-1-1V1c0-.55.45-1 1-1h10c.55 0 1 .45 1 1zm-1 10H1v2h2v-1h3v1h5v-2zm0-10H2v9h9V1z"/></svg>
      <svg height="16" width="16" class="octicon octicon-project flex-shrink-0 js-jump-to-octicon-project d-none" title="Project" aria-label="Project" viewBox="0 0 15 16" version="1.1" role="img"><path fill-rule="evenodd" d="M10 12h3V2h-3v10zm-4-2h3V2H6v8zm-4 4h3V2H2v12zm-1 1h13V1H1v14zM14 0H1a1 1 0 0 0-1 1v14a1 1 0 0 0 1 1h13a1 1 0 0 0 1-1V1a1 1 0 0 0-1-1z"/></svg>
      <svg height="16" width="16" class="octicon octicon-search flex-shrink-0 js-jump-to-octicon-search d-none" title="Search" aria-label="Search" viewBox="0 0 16 16" version="1.1" role="img"><path fill-rule="evenodd" d="M15.7 13.3l-3.81-3.83A5.93 5.93 0 0 0 13 6c0-3.31-2.69-6-6-6S1 2.69 1 6s2.69 6 6 6c1.3 0 2.48-.41 3.47-1.11l3.83 3.81c.19.2.45.3.7.3.25 0 .52-.09.7-.3a.996.996 0 0 0 0-1.41v.01zM7 10.7c-2.59 0-4.7-2.11-4.7-4.7 0-2.59 2.11-4.7 4.7-4.7 2.59 0 4.7 2.11 4.7 4.7 0 2.59-2.11 4.7-4.7 4.7z"/></svg>
    </div>

    <img class="avatar mr-2 flex-shrink-0 js-jump-to-suggestion-avatar d-none" alt="" aria-label="Team" src="" width="28" height="28">

    <div class="jump-to-suggestion-name js-jump-to-suggestion-name flex-auto overflow-hidden text-left no-wrap css-truncate css-truncate-target">
    </div>

    <div class="border rounded-1 flex-shrink-0 bg-gray px-1 text-gray-light ml-1 f6 d-none js-jump-to-badge-search">
      <span class="js-jump-to-badge-search-text-default d-none" aria-label="in this repository">
        In this repository
      </span>
      <span class="js-jump-to-badge-search-text-global d-none" aria-label="in all of GitHub">
        All GitHub
      </span>
      <span aria-hidden="true" class="d-inline-block ml-1 v-align-middle">↵</span>
    </div>

    <div aria-hidden="true" class="border rounded-1 flex-shrink-0 bg-gray px-1 text-gray-light ml-1 f6 d-none d-on-nav-focus js-jump-to-badge-jump">
      Jump to
      <span class="d-inline-block ml-1 v-align-middle">↵</span>
    </div>
  </a>
</li>


</ul>

            </div>
      </label>
</form>  </div>
</div>

          </div>

        <a class="HeaderMenu-link no-underline mr-3" href="/login?return_to=%2Frichardghirst%2FPiBits%2Fblob%2Fmaster%2FMPU6050-Pi-Demo%2FMPU6050_6Axis_MotionApps20.h" data-ga-click="(Logged out) Header, clicked Sign in, text:sign-in">Sign&nbsp;in</a>
          <a class="HeaderMenu-link d-inline-block no-underline border border-gray-dark rounded-1 px-2 py-1" href="/join" data-ga-click="(Logged out) Header, clicked Sign up, text:sign-up">Sign&nbsp;up</a>
      </div>
    </div>
  </div>
</header>

  </div>

  <div id="start-of-content" class="show-on-focus"></div>

    <div id="js-flash-container">

</div>



  <div role="main" class="application-main " data-commit-hovercards-enabled>
        <div itemscope itemtype="http://schema.org/SoftwareSourceCode" class="">
    <div id="js-repo-pjax-container" data-pjax-container >
      


  





  <div class="pagehead repohead instapaper_ignore readability-menu experiment-repo-nav  ">
    <div class="repohead-details-container clearfix container">

      <ul class="pagehead-actions">

  <li>
      <a href="/login?return_to=%2Frichardghirst%2FPiBits"
    class="btn btn-sm btn-with-count tooltipped tooltipped-s"
    aria-label="You must be signed in to watch a repository" rel="nofollow">
    <svg class="octicon octicon-eye v-align-text-bottom" viewBox="0 0 16 16" version="1.1" width="16" height="16" aria-hidden="true"><path fill-rule="evenodd" d="M8.06 2C3 2 0 8 0 8s3 6 8.06 6C13 14 16 8 16 8s-3-6-7.94-6zM8 12c-2.2 0-4-1.78-4-4 0-2.2 1.8-4 4-4 2.22 0 4 1.8 4 4 0 2.22-1.78 4-4 4zm2-4c0 1.11-.89 2-2 2-1.11 0-2-.89-2-2 0-1.11.89-2 2-2 1.11 0 2 .89 2 2z"/></svg>
    Watch
  </a>
  <a class="social-count" href="/richardghirst/PiBits/watchers"
     aria-label="94 users are watching this repository">
    94
  </a>

  </li>

  <li>
        <a href="/login?return_to=%2Frichardghirst%2FPiBits"
      class="btn btn-sm btn-with-count tooltipped tooltipped-s"
      aria-label="You must be signed in to star a repository" rel="nofollow">
      <svg class="octicon octicon-star v-align-text-bottom" viewBox="0 0 14 16" version="1.1" width="14" height="16" aria-hidden="true"><path fill-rule="evenodd" d="M14 6l-4.9-.64L7 1 4.9 5.36 0 6l3.6 3.26L2.67 14 7 11.67 11.33 14l-.93-4.74L14 6z"/></svg>
      Star
    </a>

    <a class="social-count js-social-count" href="/richardghirst/PiBits/stargazers"
      aria-label="690 users starred this repository">
      690
    </a>

  </li>

  <li>
      <a href="/login?return_to=%2Frichardghirst%2FPiBits"
        class="btn btn-sm btn-with-count tooltipped tooltipped-s"
        aria-label="You must be signed in to fork a repository" rel="nofollow">
        <svg class="octicon octicon-repo-forked v-align-text-bottom" viewBox="0 0 10 16" version="1.1" width="10" height="16" aria-hidden="true"><path fill-rule="evenodd" d="M8 1a1.993 1.993 0 0 0-1 3.72V6L5 8 3 6V4.72A1.993 1.993 0 0 0 2 1a1.993 1.993 0 0 0-1 3.72V6.5l3 3v1.78A1.993 1.993 0 0 0 5 15a1.993 1.993 0 0 0 1-3.72V9.5l3-3V4.72A1.993 1.993 0 0 0 8 1zM2 4.2C1.34 4.2.8 3.65.8 3c0-.65.55-1.2 1.2-1.2.65 0 1.2.55 1.2 1.2 0 .65-.55 1.2-1.2 1.2zm3 10c-.66 0-1.2-.55-1.2-1.2 0-.65.55-1.2 1.2-1.2.65 0 1.2.55 1.2 1.2 0 .65-.55 1.2-1.2 1.2zm3-10c-.66 0-1.2-.55-1.2-1.2 0-.65.55-1.2 1.2-1.2.65 0 1.2.55 1.2 1.2 0 .65-.55 1.2-1.2 1.2z"/></svg>
        Fork
      </a>

    <a href="/richardghirst/PiBits/network/members" class="social-count"
       aria-label="498 users forked this repository">
      498
    </a>
  </li>
</ul>

      <h1 class="public ">
  <svg class="octicon octicon-repo" viewBox="0 0 12 16" version="1.1" width="12" height="16" aria-hidden="true"><path fill-rule="evenodd" d="M4 9H3V8h1v1zm0-3H3v1h1V6zm0-2H3v1h1V4zm0-2H3v1h1V2zm8-1v12c0 .55-.45 1-1 1H6v2l-1.5-1.5L3 16v-2H1c-.55 0-1-.45-1-1V1c0-.55.45-1 1-1h10c.55 0 1 .45 1 1zm-1 10H1v2h2v-1h3v1h5v-2zm0-10H2v9h9V1z"/></svg>
  <span class="author" itemprop="author"><a class="url fn" rel="author" data-hovercard-type="user" data-hovercard-url="/hovercards?user_id=1824945" data-octo-click="hovercard-link-click" data-octo-dimensions="link_type:self" href="/richardghirst">richardghirst</a></span><!--
--><span class="path-divider">/</span><!--
--><strong itemprop="name"><a data-pjax="#js-repo-pjax-container" href="/richardghirst/PiBits">PiBits</a></strong>

</h1>

    </div>
    
<nav class="reponav js-repo-nav js-sidenav-container-pjax container"
     itemscope
     itemtype="http://schema.org/BreadcrumbList"
    aria-label="Repository"
     data-pjax="#js-repo-pjax-container">

  <span itemscope itemtype="http://schema.org/ListItem" itemprop="itemListElement">
    <a class="js-selected-navigation-item selected reponav-item" itemprop="url" data-hotkey="g c" aria-current="page" data-selected-links="repo_source repo_downloads repo_commits repo_releases repo_tags repo_branches repo_packages /richardghirst/PiBits" href="/richardghirst/PiBits">
      <svg class="octicon octicon-code" viewBox="0 0 14 16" version="1.1" width="14" height="16" aria-hidden="true"><path fill-rule="evenodd" d="M9.5 3L8 4.5 11.5 8 8 11.5 9.5 13 14 8 9.5 3zm-5 0L0 8l4.5 5L6 11.5 2.5 8 6 4.5 4.5 3z"/></svg>
      <span itemprop="name">Code</span>
      <meta itemprop="position" content="1">
</a>  </span>

    <span itemscope itemtype="http://schema.org/ListItem" itemprop="itemListElement">
      <a itemprop="url" data-hotkey="g i" class="js-selected-navigation-item reponav-item" data-selected-links="repo_issues repo_labels repo_milestones /richardghirst/PiBits/issues" href="/richardghirst/PiBits/issues">
        <svg class="octicon octicon-issue-opened" viewBox="0 0 14 16" version="1.1" width="14" height="16" aria-hidden="true"><path fill-rule="evenodd" d="M7 2.3c3.14 0 5.7 2.56 5.7 5.7s-2.56 5.7-5.7 5.7A5.71 5.71 0 0 1 1.3 8c0-3.14 2.56-5.7 5.7-5.7zM7 1C3.14 1 0 4.14 0 8s3.14 7 7 7 7-3.14 7-7-3.14-7-7-7zm1 3H6v5h2V4zm0 6H6v2h2v-2z"/></svg>
        <span itemprop="name">Issues</span>
        <span class="Counter">46</span>
        <meta itemprop="position" content="2">
</a>    </span>

  <span itemscope itemtype="http://schema.org/ListItem" itemprop="itemListElement">
    <a data-hotkey="g p" itemprop="url" class="js-selected-navigation-item reponav-item" data-selected-links="repo_pulls checks /richardghirst/PiBits/pulls" href="/richardghirst/PiBits/pulls">
      <svg class="octicon octicon-git-pull-request" viewBox="0 0 12 16" version="1.1" width="12" height="16" aria-hidden="true"><path fill-rule="evenodd" d="M11 11.28V5c-.03-.78-.34-1.47-.94-2.06C9.46 2.35 8.78 2.03 8 2H7V0L4 3l3 3V4h1c.27.02.48.11.69.31.21.2.3.42.31.69v6.28A1.993 1.993 0 0 0 10 15a1.993 1.993 0 0 0 1-3.72zm-1 2.92c-.66 0-1.2-.55-1.2-1.2 0-.65.55-1.2 1.2-1.2.65 0 1.2.55 1.2 1.2 0 .65-.55 1.2-1.2 1.2zM4 3c0-1.11-.89-2-2-2a1.993 1.993 0 0 0-1 3.72v6.56A1.993 1.993 0 0 0 2 15a1.993 1.993 0 0 0 1-3.72V4.72c.59-.34 1-.98 1-1.72zm-.8 10c0 .66-.55 1.2-1.2 1.2-.65 0-1.2-.55-1.2-1.2 0-.65.55-1.2 1.2-1.2.65 0 1.2.55 1.2 1.2zM2 4.2C1.34 4.2.8 3.65.8 3c0-.65.55-1.2 1.2-1.2.65 0 1.2.55 1.2 1.2 0 .65-.55 1.2-1.2 1.2z"/></svg>
      <span itemprop="name">Pull requests</span>
      <span class="Counter">7</span>
      <meta itemprop="position" content="3">
</a>  </span>


    <a data-hotkey="g b" class="js-selected-navigation-item reponav-item" data-selected-links="repo_projects new_repo_project repo_project /richardghirst/PiBits/projects" href="/richardghirst/PiBits/projects">
      <svg class="octicon octicon-project" viewBox="0 0 15 16" version="1.1" width="15" height="16" aria-hidden="true"><path fill-rule="evenodd" d="M10 12h3V2h-3v10zm-4-2h3V2H6v8zm-4 4h3V2H2v12zm-1 1h13V1H1v14zM14 0H1a1 1 0 0 0-1 1v14a1 1 0 0 0 1 1h13a1 1 0 0 0 1-1V1a1 1 0 0 0-1-1z"/></svg>
      Projects
      <span class="Counter" >0</span>
</a>


    <a class="js-selected-navigation-item reponav-item" data-selected-links="repo_graphs repo_contributors dependency_graph pulse alerts security people /richardghirst/PiBits/pulse" href="/richardghirst/PiBits/pulse">
      <svg class="octicon octicon-graph" viewBox="0 0 16 16" version="1.1" width="16" height="16" aria-hidden="true"><path fill-rule="evenodd" d="M16 14v1H0V0h1v14h15zM5 13H3V8h2v5zm4 0H7V3h2v10zm4 0h-2V6h2v7z"/></svg>
      Insights
</a>

</nav>


  </div>
<div class="container new-discussion-timeline experiment-repo-nav  ">
  <div class="repository-content ">

    
    



  
    <a class="d-none js-permalink-shortcut" data-hotkey="y" href="/richardghirst/PiBits/blob/4309bab2cae48600e755452a15c89a7ab3f03dfb/MPU6050-Pi-Demo/MPU6050_6Axis_MotionApps20.h">Permalink</a>

    <!-- blob contrib key: blob_contributors:v21:37d95bd8d00d56306239c863a79798cc -->

        <div class="signup-prompt-bg rounded-1">
      <div class="signup-prompt p-4 text-center mb-4 rounded-1">
        <div class="position-relative">
          <!-- '"` --><!-- </textarea></xmp> --></option></form><form action="/site/dismiss_signup_prompt" accept-charset="UTF-8" method="post"><input name="utf8" type="hidden" value="&#x2713;" /><input type="hidden" name="authenticity_token" value="LAltlyAGJpchctdlGDdmdCtJVGIt5Etl5SyL6IRzTArTBPtmW3fya53JDoSVe30kb82tNnwrz5hG2vu0NprhXg==" />
            <button type="submit" class="position-absolute top-0 right-0 btn-link link-gray" data-ga-click="(Logged out) Sign up prompt, clicked Dismiss, text:dismiss">
              Dismiss
            </button>
</form>          <h3 class="pt-2">Join GitHub today</h3>
          <p class="col-6 mx-auto">GitHub is home to over 31 million developers working together to host and review code, manage projects, and build software together.</p>
          <a class="btn btn-primary" href="/join?source=prompt-blob-show" data-ga-click="(Logged out) Sign up prompt, clicked Sign up, text:sign-up">Sign up</a>
        </div>
      </div>
    </div>


    <div class="file-navigation">
      
<details class="details-reset details-overlay select-menu branch-select-menu float-left">
  <summary class="btn btn-sm select-menu-button css-truncate"
           data-hotkey="w"
           
           title="Switch branches or tags">
    <i>Branch:</i>
    <span class="css-truncate-target">master</span>
  </summary>

  <details-menu class="select-menu-modal position-absolute" style="z-index: 99;" src="/richardghirst/PiBits/ref-list/master/MPU6050-Pi-Demo/MPU6050_6Axis_MotionApps20.h?source_action=show&amp;source_controller=blob" preload>
    <include-fragment class="select-menu-loading-overlay anim-pulse">
      <svg height="32" class="octicon octicon-octoface" viewBox="0 0 16 16" version="1.1" width="32" aria-hidden="true"><path fill-rule="evenodd" d="M14.7 5.34c.13-.32.55-1.59-.13-3.31 0 0-1.05-.33-3.44 1.3-1-.28-2.07-.32-3.13-.32s-2.13.04-3.13.32c-2.39-1.64-3.44-1.3-3.44-1.3-.68 1.72-.26 2.99-.13 3.31C.49 6.21 0 7.33 0 8.69 0 13.84 3.33 15 7.98 15S16 13.84 16 8.69c0-1.36-.49-2.48-1.3-3.35zM8 14.02c-3.3 0-5.98-.15-5.98-3.35 0-.76.38-1.48 1.02-2.07 1.07-.98 2.9-.46 4.96-.46 2.07 0 3.88-.52 4.96.46.65.59 1.02 1.3 1.02 2.07 0 3.19-2.68 3.35-5.98 3.35zM5.49 9.01c-.66 0-1.2.8-1.2 1.78s.54 1.79 1.2 1.79c.66 0 1.2-.8 1.2-1.79s-.54-1.78-1.2-1.78zm5.02 0c-.66 0-1.2.79-1.2 1.78s.54 1.79 1.2 1.79c.66 0 1.2-.8 1.2-1.79s-.53-1.78-1.2-1.78z"/></svg>
    </include-fragment>
  </details-menu>
</details>

      <div class="BtnGroup float-right">
        <a href="/richardghirst/PiBits/find/master"
              class="js-pjax-capture-input btn btn-sm BtnGroup-item"
              data-pjax
              data-hotkey="t">
          Find file
        </a>
        <clipboard-copy for="blob-path" class="btn btn-sm BtnGroup-item">
          Copy path
        </clipboard-copy>
      </div>
      <div id="blob-path" class="breadcrumb">
        <span class="repo-root js-repo-root"><span class="js-path-segment"><a data-pjax="true" href="/richardghirst/PiBits"><span>PiBits</span></a></span></span><span class="separator">/</span><span class="js-path-segment"><a data-pjax="true" href="/richardghirst/PiBits/tree/master/MPU6050-Pi-Demo"><span>MPU6050-Pi-Demo</span></a></span><span class="separator">/</span><strong class="final-path">MPU6050_6Axis_MotionApps20.h</strong>
      </div>
    </div>


    
  <div class="commit-tease">
      <span class="float-right">
        <a class="commit-tease-sha" href="/richardghirst/PiBits/commit/0a349ee6001e61919905182fc9de367c095ebe23" data-pjax>
          0a349ee
        </a>
        <relative-time datetime="2013-12-22T23:53:34Z">Dec 23, 2013</relative-time>
      </span>
      <div>
        <a rel="contributor" data-skip-pjax="true" data-hovercard-type="user" data-hovercard-url="/hovercards?user_id=1710551" data-octo-click="hovercard-link-click" data-octo-dimensions="link_type:self" href="/L31N"><img class="avatar" src="https://avatars1.githubusercontent.com/u/1710551?s=40&amp;v=4" width="20" height="20" alt="@L31N" /></a>
        <a class="user-mention" rel="contributor" data-hovercard-type="user" data-hovercard-url="/hovercards?user_id=1710551" data-octo-click="hovercard-link-click" data-octo-dimensions="link_type:self" href="/L31N">L31N</a>
          <a data-pjax="true" title="fixed bug in MPU6050_6Axis_MotionApps20.h function uint8_t MPU6050::dmpInitialize() which caused an i2c-read error in case of an empty fifoCount." class="message" href="/richardghirst/PiBits/commit/0a349ee6001e61919905182fc9de367c095ebe23">fixed bug in MPU6050_6Axis_MotionApps20.h function uint8_t MPU6050::d…</a>
      </div>

    <div class="commit-tease-contributors">
      
<details class="details-reset details-overlay details-overlay-dark lh-default text-gray-dark float-left mr-2" id="blob_contributors_box">
  <summary
      class="btn-link"
      aria-haspopup="dialog"
      
      
      >
    
    <span><strong>1</strong> contributor</span>
  </summary>
  <details-dialog class="Box Box--overlay d-flex flex-column anim-fade-in fast " aria-label="Users who have contributed to this file">
    <div class="Box-header">
      <button class="Box-btn-octicon btn-octicon float-right" type="button" aria-label="Close dialog" data-close-dialog>
        <svg class="octicon octicon-x" viewBox="0 0 12 16" version="1.1" width="12" height="16" aria-hidden="true"><path fill-rule="evenodd" d="M7.48 8l3.75 3.75-1.48 1.48L6 9.48l-3.75 3.75-1.48-1.48L4.52 8 .77 4.25l1.48-1.48L6 6.52l3.75-3.75 1.48 1.48L7.48 8z"/></svg>
      </button>
      <h3 class="Box-title">Users who have contributed to this file</h3>
    </div>
    
        <ul class="list-style-none overflow-auto">
            <li class="Box-row">
              <a class="link-gray-dark no-underline" href="/L31N">
                <img class="avatar mr-2" alt="" src="https://avatars1.githubusercontent.com/u/1710551?s=40&amp;v=4" width="20" height="20" />
                L31N
</a>            </li>
        </ul>

  </details-dialog>
</details>
      
    </div>
  </div>





    <div class="file ">
      
<div class="file-header">

  <div class="file-actions">

    <div class="BtnGroup">
      <a id="raw-url" class="btn btn-sm BtnGroup-item" href="/richardghirst/PiBits/raw/master/MPU6050-Pi-Demo/MPU6050_6Axis_MotionApps20.h">Raw</a>
        <a class="btn btn-sm js-update-url-with-hash BtnGroup-item" data-hotkey="b" href="/richardghirst/PiBits/blame/master/MPU6050-Pi-Demo/MPU6050_6Axis_MotionApps20.h">Blame</a>
      <a rel="nofollow" class="btn btn-sm BtnGroup-item" href="/richardghirst/PiBits/commits/master/MPU6050-Pi-Demo/MPU6050_6Axis_MotionApps20.h">History</a>
    </div>



        <button type="button" class="btn-octicon disabled tooltipped tooltipped-nw"
          aria-label="You must be signed in to make or propose changes">
          <svg class="octicon octicon-pencil" viewBox="0 0 14 16" version="1.1" width="14" height="16" aria-hidden="true"><path fill-rule="evenodd" d="M0 12v3h3l8-8-3-3-8 8zm3 2H1v-2h1v1h1v1zm10.3-9.3L12 6 9 3l1.3-1.3a.996.996 0 0 1 1.41 0l1.59 1.59c.39.39.39 1.02 0 1.41z"/></svg>
        </button>
        <button type="button" class="btn-octicon btn-octicon-danger disabled tooltipped tooltipped-nw"
          aria-label="You must be signed in to make or propose changes">
          <svg class="octicon octicon-trashcan" viewBox="0 0 12 16" version="1.1" width="12" height="16" aria-hidden="true"><path fill-rule="evenodd" d="M11 2H9c0-.55-.45-1-1-1H5c-.55 0-1 .45-1 1H2c-.55 0-1 .45-1 1v1c0 .55.45 1 1 1v9c0 .55.45 1 1 1h7c.55 0 1-.45 1-1V5c.55 0 1-.45 1-1V3c0-.55-.45-1-1-1zm-1 12H3V5h1v8h1V5h1v8h1V5h1v8h1V5h1v9zm1-10H2V3h9v1z"/></svg>
        </button>
  </div>

  <div class="file-info">
      710 lines (617 sloc)
      <span class="file-info-divider"></span>
    38.2 KB
  </div>
</div>

      

  <div itemprop="text" class="blob-wrapper data type-c ">
      
<table class="highlight tab-size js-file-line-container" data-tab-size="8">
      <tr>
        <td id="L1" class="blob-num js-line-number" data-line-number="1"></td>
        <td id="LC1" class="blob-code blob-code-inner js-file-line"><span class="pl-c"><span class="pl-c">//</span> I2Cdev library collection - MPU6050 I2C device class, 6-axis MotionApps 2.0 implementation</span></td>
      </tr>
      <tr>
        <td id="L2" class="blob-num js-line-number" data-line-number="2"></td>
        <td id="LC2" class="blob-code blob-code-inner js-file-line"><span class="pl-c"><span class="pl-c">//</span> Based on InvenSense MPU-6050 register map document rev. 2.0, 5/19/2011 (RM-MPU-6000A-00)</span></td>
      </tr>
      <tr>
        <td id="L3" class="blob-num js-line-number" data-line-number="3"></td>
        <td id="LC3" class="blob-code blob-code-inner js-file-line"><span class="pl-c"><span class="pl-c">//</span> 6/18/2012 by Jeff Rowberg &lt;jeff@rowberg.net&gt;</span></td>
      </tr>
      <tr>
        <td id="L4" class="blob-num js-line-number" data-line-number="4"></td>
        <td id="LC4" class="blob-code blob-code-inner js-file-line"><span class="pl-c"><span class="pl-c">//</span> Updates should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib</span></td>
      </tr>
      <tr>
        <td id="L5" class="blob-num js-line-number" data-line-number="5"></td>
        <td id="LC5" class="blob-code blob-code-inner js-file-line"><span class="pl-c"><span class="pl-c">//</span></span></td>
      </tr>
      <tr>
        <td id="L6" class="blob-num js-line-number" data-line-number="6"></td>
        <td id="LC6" class="blob-code blob-code-inner js-file-line"><span class="pl-c"><span class="pl-c">//</span> Changelog:</span></td>
      </tr>
      <tr>
        <td id="L7" class="blob-num js-line-number" data-line-number="7"></td>
        <td id="LC7" class="blob-code blob-code-inner js-file-line"><span class="pl-c"><span class="pl-c">//</span>     ... - ongoing debug release</span></td>
      </tr>
      <tr>
        <td id="L8" class="blob-num js-line-number" data-line-number="8"></td>
        <td id="LC8" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L9" class="blob-num js-line-number" data-line-number="9"></td>
        <td id="LC9" class="blob-code blob-code-inner js-file-line"><span class="pl-c"><span class="pl-c">/*</span> ============================================</span></td>
      </tr>
      <tr>
        <td id="L10" class="blob-num js-line-number" data-line-number="10"></td>
        <td id="LC10" class="blob-code blob-code-inner js-file-line"><span class="pl-c">I2Cdev device library code is placed under the MIT license</span></td>
      </tr>
      <tr>
        <td id="L11" class="blob-num js-line-number" data-line-number="11"></td>
        <td id="LC11" class="blob-code blob-code-inner js-file-line"><span class="pl-c">Copyright (c) 2012 Jeff Rowberg</span></td>
      </tr>
      <tr>
        <td id="L12" class="blob-num js-line-number" data-line-number="12"></td>
        <td id="LC12" class="blob-code blob-code-inner js-file-line"><span class="pl-c"></span></td>
      </tr>
      <tr>
        <td id="L13" class="blob-num js-line-number" data-line-number="13"></td>
        <td id="LC13" class="blob-code blob-code-inner js-file-line"><span class="pl-c">Permission is hereby granted, free of charge, to any person obtaining a copy</span></td>
      </tr>
      <tr>
        <td id="L14" class="blob-num js-line-number" data-line-number="14"></td>
        <td id="LC14" class="blob-code blob-code-inner js-file-line"><span class="pl-c">of this software and associated documentation files (the &quot;Software&quot;), to deal</span></td>
      </tr>
      <tr>
        <td id="L15" class="blob-num js-line-number" data-line-number="15"></td>
        <td id="LC15" class="blob-code blob-code-inner js-file-line"><span class="pl-c">in the Software without restriction, including without limitation the rights</span></td>
      </tr>
      <tr>
        <td id="L16" class="blob-num js-line-number" data-line-number="16"></td>
        <td id="LC16" class="blob-code blob-code-inner js-file-line"><span class="pl-c">to use, copy, modify, merge, publish, distribute, sublicense, and/or sell</span></td>
      </tr>
      <tr>
        <td id="L17" class="blob-num js-line-number" data-line-number="17"></td>
        <td id="LC17" class="blob-code blob-code-inner js-file-line"><span class="pl-c">copies of the Software, and to permit persons to whom the Software is</span></td>
      </tr>
      <tr>
        <td id="L18" class="blob-num js-line-number" data-line-number="18"></td>
        <td id="LC18" class="blob-code blob-code-inner js-file-line"><span class="pl-c">furnished to do so, subject to the following conditions:</span></td>
      </tr>
      <tr>
        <td id="L19" class="blob-num js-line-number" data-line-number="19"></td>
        <td id="LC19" class="blob-code blob-code-inner js-file-line"><span class="pl-c"></span></td>
      </tr>
      <tr>
        <td id="L20" class="blob-num js-line-number" data-line-number="20"></td>
        <td id="LC20" class="blob-code blob-code-inner js-file-line"><span class="pl-c">The above copyright notice and this permission notice shall be included in</span></td>
      </tr>
      <tr>
        <td id="L21" class="blob-num js-line-number" data-line-number="21"></td>
        <td id="LC21" class="blob-code blob-code-inner js-file-line"><span class="pl-c">all copies or substantial portions of the Software.</span></td>
      </tr>
      <tr>
        <td id="L22" class="blob-num js-line-number" data-line-number="22"></td>
        <td id="LC22" class="blob-code blob-code-inner js-file-line"><span class="pl-c"></span></td>
      </tr>
      <tr>
        <td id="L23" class="blob-num js-line-number" data-line-number="23"></td>
        <td id="LC23" class="blob-code blob-code-inner js-file-line"><span class="pl-c">THE SOFTWARE IS PROVIDED &quot;AS IS&quot;, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR</span></td>
      </tr>
      <tr>
        <td id="L24" class="blob-num js-line-number" data-line-number="24"></td>
        <td id="LC24" class="blob-code blob-code-inner js-file-line"><span class="pl-c">IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,</span></td>
      </tr>
      <tr>
        <td id="L25" class="blob-num js-line-number" data-line-number="25"></td>
        <td id="LC25" class="blob-code blob-code-inner js-file-line"><span class="pl-c">FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE</span></td>
      </tr>
      <tr>
        <td id="L26" class="blob-num js-line-number" data-line-number="26"></td>
        <td id="LC26" class="blob-code blob-code-inner js-file-line"><span class="pl-c">AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER</span></td>
      </tr>
      <tr>
        <td id="L27" class="blob-num js-line-number" data-line-number="27"></td>
        <td id="LC27" class="blob-code blob-code-inner js-file-line"><span class="pl-c">LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,</span></td>
      </tr>
      <tr>
        <td id="L28" class="blob-num js-line-number" data-line-number="28"></td>
        <td id="LC28" class="blob-code blob-code-inner js-file-line"><span class="pl-c">OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN</span></td>
      </tr>
      <tr>
        <td id="L29" class="blob-num js-line-number" data-line-number="29"></td>
        <td id="LC29" class="blob-code blob-code-inner js-file-line"><span class="pl-c">THE SOFTWARE.</span></td>
      </tr>
      <tr>
        <td id="L30" class="blob-num js-line-number" data-line-number="30"></td>
        <td id="LC30" class="blob-code blob-code-inner js-file-line"><span class="pl-c">===============================================</span></td>
      </tr>
      <tr>
        <td id="L31" class="blob-num js-line-number" data-line-number="31"></td>
        <td id="LC31" class="blob-code blob-code-inner js-file-line"><span class="pl-c"><span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L32" class="blob-num js-line-number" data-line-number="32"></td>
        <td id="LC32" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L33" class="blob-num js-line-number" data-line-number="33"></td>
        <td id="LC33" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">ifndef</span> _MPU6050_6AXIS_MOTIONAPPS20_H_</td>
      </tr>
      <tr>
        <td id="L34" class="blob-num js-line-number" data-line-number="34"></td>
        <td id="LC34" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">_MPU6050_6AXIS_MOTIONAPPS20_H_</span></td>
      </tr>
      <tr>
        <td id="L35" class="blob-num js-line-number" data-line-number="35"></td>
        <td id="LC35" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L36" class="blob-num js-line-number" data-line-number="36"></td>
        <td id="LC36" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">include</span> <span class="pl-s"><span class="pl-pds">&quot;</span>I2Cdev.h<span class="pl-pds">&quot;</span></span></td>
      </tr>
      <tr>
        <td id="L37" class="blob-num js-line-number" data-line-number="37"></td>
        <td id="LC37" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">include</span> <span class="pl-s"><span class="pl-pds">&quot;</span>helper_3dmath.h<span class="pl-pds">&quot;</span></span></td>
      </tr>
      <tr>
        <td id="L38" class="blob-num js-line-number" data-line-number="38"></td>
        <td id="LC38" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L39" class="blob-num js-line-number" data-line-number="39"></td>
        <td id="LC39" class="blob-code blob-code-inner js-file-line"><span class="pl-c"><span class="pl-c">//</span> MotionApps 2.0 DMP implementation, built using the MPU-6050EVB evaluation board</span></td>
      </tr>
      <tr>
        <td id="L40" class="blob-num js-line-number" data-line-number="40"></td>
        <td id="LC40" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">MPU6050_INCLUDE_DMP_MOTIONAPPS20</span></td>
      </tr>
      <tr>
        <td id="L41" class="blob-num js-line-number" data-line-number="41"></td>
        <td id="LC41" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L42" class="blob-num js-line-number" data-line-number="42"></td>
        <td id="LC42" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">include</span> <span class="pl-s"><span class="pl-pds">&quot;</span>MPU6050.h<span class="pl-pds">&quot;</span></span></td>
      </tr>
      <tr>
        <td id="L43" class="blob-num js-line-number" data-line-number="43"></td>
        <td id="LC43" class="blob-code blob-code-inner js-file-line"><span class="pl-c"><span class="pl-c">//</span>#include &lt;avr/pgmspace.h&gt;</span></td>
      </tr>
      <tr>
        <td id="L44" class="blob-num js-line-number" data-line-number="44"></td>
        <td id="LC44" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L45" class="blob-num js-line-number" data-line-number="45"></td>
        <td id="LC45" class="blob-code blob-code-inner js-file-line"><span class="pl-c"><span class="pl-c">/*</span> Source is from the InvenSense MotionApps v2 demo code. Original source is</span></td>
      </tr>
      <tr>
        <td id="L46" class="blob-num js-line-number" data-line-number="46"></td>
        <td id="LC46" class="blob-code blob-code-inner js-file-line"><span class="pl-c"> * unavailable, unless you happen to be amazing as decompiling binary by</span></td>
      </tr>
      <tr>
        <td id="L47" class="blob-num js-line-number" data-line-number="47"></td>
        <td id="LC47" class="blob-code blob-code-inner js-file-line"><span class="pl-c"> * hand (in which case, please contact me, and I&#39;m totally serious).</span></td>
      </tr>
      <tr>
        <td id="L48" class="blob-num js-line-number" data-line-number="48"></td>
        <td id="LC48" class="blob-code blob-code-inner js-file-line"><span class="pl-c"> *</span></td>
      </tr>
      <tr>
        <td id="L49" class="blob-num js-line-number" data-line-number="49"></td>
        <td id="LC49" class="blob-code blob-code-inner js-file-line"><span class="pl-c"> * Also, I&#39;d like to offer many, many thanks to Noah Zerkin for all of the</span></td>
      </tr>
      <tr>
        <td id="L50" class="blob-num js-line-number" data-line-number="50"></td>
        <td id="LC50" class="blob-code blob-code-inner js-file-line"><span class="pl-c"> * DMP reverse-engineering he did to help make this bit of wizardry</span></td>
      </tr>
      <tr>
        <td id="L51" class="blob-num js-line-number" data-line-number="51"></td>
        <td id="LC51" class="blob-code blob-code-inner js-file-line"><span class="pl-c"> * possible.</span></td>
      </tr>
      <tr>
        <td id="L52" class="blob-num js-line-number" data-line-number="52"></td>
        <td id="LC52" class="blob-code blob-code-inner js-file-line"><span class="pl-c"> <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L53" class="blob-num js-line-number" data-line-number="53"></td>
        <td id="LC53" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L54" class="blob-num js-line-number" data-line-number="54"></td>
        <td id="LC54" class="blob-code blob-code-inner js-file-line"><span class="pl-c"><span class="pl-c">//</span> NOTE! Enabling DEBUG adds about 3.3kB to the flash program size.</span></td>
      </tr>
      <tr>
        <td id="L55" class="blob-num js-line-number" data-line-number="55"></td>
        <td id="LC55" class="blob-code blob-code-inner js-file-line"><span class="pl-c"><span class="pl-c">//</span> Debug output is now working even on ATMega328P MCUs (e.g. Arduino Uno)</span></td>
      </tr>
      <tr>
        <td id="L56" class="blob-num js-line-number" data-line-number="56"></td>
        <td id="LC56" class="blob-code blob-code-inner js-file-line"><span class="pl-c"><span class="pl-c">//</span> after moving string constants to flash memory storage using the F()</span></td>
      </tr>
      <tr>
        <td id="L57" class="blob-num js-line-number" data-line-number="57"></td>
        <td id="LC57" class="blob-code blob-code-inner js-file-line"><span class="pl-c"><span class="pl-c">//</span> compiler macro (Arduino IDE 1.0+ required).</span></td>
      </tr>
      <tr>
        <td id="L58" class="blob-num js-line-number" data-line-number="58"></td>
        <td id="LC58" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L59" class="blob-num js-line-number" data-line-number="59"></td>
        <td id="LC59" class="blob-code blob-code-inner js-file-line"><span class="pl-c"><span class="pl-c">//</span>#define DEBUG</span></td>
      </tr>
      <tr>
        <td id="L60" class="blob-num js-line-number" data-line-number="60"></td>
        <td id="LC60" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">ifdef</span> DEBUG</td>
      </tr>
      <tr>
        <td id="L61" class="blob-num js-line-number" data-line-number="61"></td>
        <td id="LC61" class="blob-code blob-code-inner js-file-line">    #<span class="pl-k">define</span> <span class="pl-en">DEBUG_PRINT</span>(<span class="pl-v">x</span>) Serial.print(x)</td>
      </tr>
      <tr>
        <td id="L62" class="blob-num js-line-number" data-line-number="62"></td>
        <td id="LC62" class="blob-code blob-code-inner js-file-line">    #<span class="pl-k">define</span> <span class="pl-en">DEBUG_PRINTF</span>(<span class="pl-v">x, y</span>) Serial.print(x, y)</td>
      </tr>
      <tr>
        <td id="L63" class="blob-num js-line-number" data-line-number="63"></td>
        <td id="LC63" class="blob-code blob-code-inner js-file-line">    #<span class="pl-k">define</span> <span class="pl-en">DEBUG_PRINTLN</span>(<span class="pl-v">x</span>) Serial.println(x)</td>
      </tr>
      <tr>
        <td id="L64" class="blob-num js-line-number" data-line-number="64"></td>
        <td id="LC64" class="blob-code blob-code-inner js-file-line">    #<span class="pl-k">define</span> <span class="pl-en">DEBUG_PRINTLNF</span>(<span class="pl-v">x, y</span>) Serial.println(x, y)</td>
      </tr>
      <tr>
        <td id="L65" class="blob-num js-line-number" data-line-number="65"></td>
        <td id="LC65" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">else</span></td>
      </tr>
      <tr>
        <td id="L66" class="blob-num js-line-number" data-line-number="66"></td>
        <td id="LC66" class="blob-code blob-code-inner js-file-line">    #<span class="pl-k">define</span> <span class="pl-en">DEBUG_PRINT</span>(<span class="pl-v">x</span>)</td>
      </tr>
      <tr>
        <td id="L67" class="blob-num js-line-number" data-line-number="67"></td>
        <td id="LC67" class="blob-code blob-code-inner js-file-line">    #<span class="pl-k">define</span> <span class="pl-en">DEBUG_PRINTF</span>(<span class="pl-v">x, y</span>)</td>
      </tr>
      <tr>
        <td id="L68" class="blob-num js-line-number" data-line-number="68"></td>
        <td id="LC68" class="blob-code blob-code-inner js-file-line">    #<span class="pl-k">define</span> <span class="pl-en">DEBUG_PRINTLN</span>(<span class="pl-v">x</span>)</td>
      </tr>
      <tr>
        <td id="L69" class="blob-num js-line-number" data-line-number="69"></td>
        <td id="LC69" class="blob-code blob-code-inner js-file-line">    #<span class="pl-k">define</span> <span class="pl-en">DEBUG_PRINTLNF</span>(<span class="pl-v">x, y</span>)</td>
      </tr>
      <tr>
        <td id="L70" class="blob-num js-line-number" data-line-number="70"></td>
        <td id="LC70" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">endif</span></td>
      </tr>
      <tr>
        <td id="L71" class="blob-num js-line-number" data-line-number="71"></td>
        <td id="LC71" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L72" class="blob-num js-line-number" data-line-number="72"></td>
        <td id="LC72" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">MPU6050_DMP_CODE_SIZE</span>       <span class="pl-c1">1929</span>    <span class="pl-c"><span class="pl-c">//</span> dmpMemory[]</span></td>
      </tr>
      <tr>
        <td id="L73" class="blob-num js-line-number" data-line-number="73"></td>
        <td id="LC73" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">MPU6050_DMP_CONFIG_SIZE</span>     <span class="pl-c1">192</span>     <span class="pl-c"><span class="pl-c">//</span> dmpConfig[]</span></td>
      </tr>
      <tr>
        <td id="L74" class="blob-num js-line-number" data-line-number="74"></td>
        <td id="LC74" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">MPU6050_DMP_UPDATES_SIZE</span>    <span class="pl-c1">47</span>      <span class="pl-c"><span class="pl-c">//</span> dmpUpdates[]</span></td>
      </tr>
      <tr>
        <td id="L75" class="blob-num js-line-number" data-line-number="75"></td>
        <td id="LC75" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L76" class="blob-num js-line-number" data-line-number="76"></td>
        <td id="LC76" class="blob-code blob-code-inner js-file-line"><span class="pl-c"><span class="pl-c">/*</span> ================================================================================================ *</span></td>
      </tr>
      <tr>
        <td id="L77" class="blob-num js-line-number" data-line-number="77"></td>
        <td id="LC77" class="blob-code blob-code-inner js-file-line"><span class="pl-c"> | Default MotionApps v2.0 42-byte FIFO packet structure:                                           |</span></td>
      </tr>
      <tr>
        <td id="L78" class="blob-num js-line-number" data-line-number="78"></td>
        <td id="LC78" class="blob-code blob-code-inner js-file-line"><span class="pl-c"> |                                                                                                  |</span></td>
      </tr>
      <tr>
        <td id="L79" class="blob-num js-line-number" data-line-number="79"></td>
        <td id="LC79" class="blob-code blob-code-inner js-file-line"><span class="pl-c"> | [QUAT W][      ][QUAT X][      ][QUAT Y][      ][QUAT Z][      ][GYRO X][      ][GYRO Y][      ] |</span></td>
      </tr>
      <tr>
        <td id="L80" class="blob-num js-line-number" data-line-number="80"></td>
        <td id="LC80" class="blob-code blob-code-inner js-file-line"><span class="pl-c"> |   0   1   2   3   4   5   6   7   8   9  10  11  12  13  14  15  16  17  18  19  20  21  22  23  |</span></td>
      </tr>
      <tr>
        <td id="L81" class="blob-num js-line-number" data-line-number="81"></td>
        <td id="LC81" class="blob-code blob-code-inner js-file-line"><span class="pl-c"> |                                                                                                  |</span></td>
      </tr>
      <tr>
        <td id="L82" class="blob-num js-line-number" data-line-number="82"></td>
        <td id="LC82" class="blob-code blob-code-inner js-file-line"><span class="pl-c"> | [GYRO Z][      ][ACC X ][      ][ACC Y ][      ][ACC Z ][      ][      ]                         |</span></td>
      </tr>
      <tr>
        <td id="L83" class="blob-num js-line-number" data-line-number="83"></td>
        <td id="LC83" class="blob-code blob-code-inner js-file-line"><span class="pl-c"> |  24  25  26  27  28  29  30  31  32  33  34  35  36  37  38  39  40  41                          |</span></td>
      </tr>
      <tr>
        <td id="L84" class="blob-num js-line-number" data-line-number="84"></td>
        <td id="LC84" class="blob-code blob-code-inner js-file-line"><span class="pl-c"> * ================================================================================================ <span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L85" class="blob-num js-line-number" data-line-number="85"></td>
        <td id="LC85" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L86" class="blob-num js-line-number" data-line-number="86"></td>
        <td id="LC86" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">prog_uchar</span> <span class="pl-c1">uint8_t</span></td>
      </tr>
      <tr>
        <td id="L87" class="blob-num js-line-number" data-line-number="87"></td>
        <td id="LC87" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">PROGMEM</span></td>
      </tr>
      <tr>
        <td id="L88" class="blob-num js-line-number" data-line-number="88"></td>
        <td id="LC88" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L89" class="blob-num js-line-number" data-line-number="89"></td>
        <td id="LC89" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L90" class="blob-num js-line-number" data-line-number="90"></td>
        <td id="LC90" class="blob-code blob-code-inner js-file-line"><span class="pl-c"><span class="pl-c">//</span> this block of memory gets written to the MPU on start-up, and it seems</span></td>
      </tr>
      <tr>
        <td id="L91" class="blob-num js-line-number" data-line-number="91"></td>
        <td id="LC91" class="blob-code blob-code-inner js-file-line"><span class="pl-c"><span class="pl-c">//</span> to be volatile memory, so it has to be done each time (it only takes ~1</span></td>
      </tr>
      <tr>
        <td id="L92" class="blob-num js-line-number" data-line-number="92"></td>
        <td id="LC92" class="blob-code blob-code-inner js-file-line"><span class="pl-c"><span class="pl-c">//</span> second though)</span></td>
      </tr>
      <tr>
        <td id="L93" class="blob-num js-line-number" data-line-number="93"></td>
        <td id="LC93" class="blob-code blob-code-inner js-file-line"><span class="pl-k">const</span> prog_uchar dmpMemory[MPU6050_DMP_CODE_SIZE] PROGMEM = {</td>
      </tr>
      <tr>
        <td id="L94" class="blob-num js-line-number" data-line-number="94"></td>
        <td id="LC94" class="blob-code blob-code-inner js-file-line">    <span class="pl-c"><span class="pl-c">//</span> bank 0, 256 bytes</span></td>
      </tr>
      <tr>
        <td id="L95" class="blob-num js-line-number" data-line-number="95"></td>
        <td id="LC95" class="blob-code blob-code-inner js-file-line">    <span class="pl-c1">0xFB</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x3E</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x0B</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x36</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x01</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x02</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x03</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>,</td>
      </tr>
      <tr>
        <td id="L96" class="blob-num js-line-number" data-line-number="96"></td>
        <td id="LC96" class="blob-code blob-code-inner js-file-line">    <span class="pl-c1">0x00</span>, <span class="pl-c1">0x65</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x54</span>, <span class="pl-c1">0xFF</span>, <span class="pl-c1">0xEF</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0xFA</span>, <span class="pl-c1">0x80</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x0B</span>, <span class="pl-c1">0x12</span>, <span class="pl-c1">0x82</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x01</span>,</td>
      </tr>
      <tr>
        <td id="L97" class="blob-num js-line-number" data-line-number="97"></td>
        <td id="LC97" class="blob-code blob-code-inner js-file-line">    <span class="pl-c1">0x00</span>, <span class="pl-c1">0x02</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>,</td>
      </tr>
      <tr>
        <td id="L98" class="blob-num js-line-number" data-line-number="98"></td>
        <td id="LC98" class="blob-code blob-code-inner js-file-line">    <span class="pl-c1">0x00</span>, <span class="pl-c1">0x28</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0xFF</span>, <span class="pl-c1">0xFF</span>, <span class="pl-c1">0x45</span>, <span class="pl-c1">0x81</span>, <span class="pl-c1">0xFF</span>, <span class="pl-c1">0xFF</span>, <span class="pl-c1">0xFA</span>, <span class="pl-c1">0x72</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>,</td>
      </tr>
      <tr>
        <td id="L99" class="blob-num js-line-number" data-line-number="99"></td>
        <td id="LC99" class="blob-code blob-code-inner js-file-line">    <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x03</span>, <span class="pl-c1">0xE8</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x01</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x01</span>, <span class="pl-c1">0x7F</span>, <span class="pl-c1">0xFF</span>, <span class="pl-c1">0xFF</span>, <span class="pl-c1">0xFE</span>, <span class="pl-c1">0x80</span>, <span class="pl-c1">0x01</span>,</td>
      </tr>
      <tr>
        <td id="L100" class="blob-num js-line-number" data-line-number="100"></td>
        <td id="LC100" class="blob-code blob-code-inner js-file-line">    <span class="pl-c1">0x00</span>, <span class="pl-c1">0x1B</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>,</td>
      </tr>
      <tr>
        <td id="L101" class="blob-num js-line-number" data-line-number="101"></td>
        <td id="LC101" class="blob-code blob-code-inner js-file-line">    <span class="pl-c1">0x00</span>, <span class="pl-c1">0x3E</span>, <span class="pl-c1">0x03</span>, <span class="pl-c1">0x30</span>, <span class="pl-c1">0x40</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x02</span>, <span class="pl-c1">0xCA</span>, <span class="pl-c1">0xE3</span>, <span class="pl-c1">0x09</span>, <span class="pl-c1">0x3E</span>, <span class="pl-c1">0x80</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>,</td>
      </tr>
      <tr>
        <td id="L102" class="blob-num js-line-number" data-line-number="102"></td>
        <td id="LC102" class="blob-code blob-code-inner js-file-line">    <span class="pl-c1">0x20</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x40</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x60</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>,</td>
      </tr>
      <tr>
        <td id="L103" class="blob-num js-line-number" data-line-number="103"></td>
        <td id="LC103" class="blob-code blob-code-inner js-file-line">    <span class="pl-c1">0x41</span>, <span class="pl-c1">0xFF</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x0B</span>, <span class="pl-c1">0x2A</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x16</span>, <span class="pl-c1">0x55</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x21</span>, <span class="pl-c1">0x82</span>,</td>
      </tr>
      <tr>
        <td id="L104" class="blob-num js-line-number" data-line-number="104"></td>
        <td id="LC104" class="blob-code blob-code-inner js-file-line">    <span class="pl-c1">0xFD</span>, <span class="pl-c1">0x87</span>, <span class="pl-c1">0x26</span>, <span class="pl-c1">0x50</span>, <span class="pl-c1">0xFD</span>, <span class="pl-c1">0x80</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x1F</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x05</span>, <span class="pl-c1">0x80</span>, <span class="pl-c1">0x00</span>,</td>
      </tr>
      <tr>
        <td id="L105" class="blob-num js-line-number" data-line-number="105"></td>
        <td id="LC105" class="blob-code blob-code-inner js-file-line">    <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x01</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x02</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x03</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>,</td>
      </tr>
      <tr>
        <td id="L106" class="blob-num js-line-number" data-line-number="106"></td>
        <td id="LC106" class="blob-code blob-code-inner js-file-line">    <span class="pl-c1">0x40</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x04</span>, <span class="pl-c1">0x6F</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x02</span>, <span class="pl-c1">0x65</span>, <span class="pl-c1">0x32</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x5E</span>, <span class="pl-c1">0xC0</span>,</td>
      </tr>
      <tr>
        <td id="L107" class="blob-num js-line-number" data-line-number="107"></td>
        <td id="LC107" class="blob-code blob-code-inner js-file-line">    <span class="pl-c1">0x40</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>,</td>
      </tr>
      <tr>
        <td id="L108" class="blob-num js-line-number" data-line-number="108"></td>
        <td id="LC108" class="blob-code blob-code-inner js-file-line">    <span class="pl-c1">0xFB</span>, <span class="pl-c1">0x8C</span>, <span class="pl-c1">0x6F</span>, <span class="pl-c1">0x5D</span>, <span class="pl-c1">0xFD</span>, <span class="pl-c1">0x5D</span>, <span class="pl-c1">0x08</span>, <span class="pl-c1">0xD9</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x7C</span>, <span class="pl-c1">0x73</span>, <span class="pl-c1">0x3B</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x6C</span>, <span class="pl-c1">0x12</span>, <span class="pl-c1">0xCC</span>,</td>
      </tr>
      <tr>
        <td id="L109" class="blob-num js-line-number" data-line-number="109"></td>
        <td id="LC109" class="blob-code blob-code-inner js-file-line">    <span class="pl-c1">0x32</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x13</span>, <span class="pl-c1">0x9D</span>, <span class="pl-c1">0x32</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0xD0</span>, <span class="pl-c1">0xD6</span>, <span class="pl-c1">0x32</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x08</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x40</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x01</span>, <span class="pl-c1">0xF4</span>,</td>
      </tr>
      <tr>
        <td id="L110" class="blob-num js-line-number" data-line-number="110"></td>
        <td id="LC110" class="blob-code blob-code-inner js-file-line">    <span class="pl-c1">0xFF</span>, <span class="pl-c1">0xE6</span>, <span class="pl-c1">0x80</span>, <span class="pl-c1">0x79</span>, <span class="pl-c1">0x02</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0xD0</span>, <span class="pl-c1">0xD6</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x27</span>, <span class="pl-c1">0x10</span>,</td>
      </tr>
      <tr>
        <td id="L111" class="blob-num js-line-number" data-line-number="111"></td>
        <td id="LC111" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L112" class="blob-num js-line-number" data-line-number="112"></td>
        <td id="LC112" class="blob-code blob-code-inner js-file-line">    <span class="pl-c"><span class="pl-c">//</span> bank 1, 256 bytes</span></td>
      </tr>
      <tr>
        <td id="L113" class="blob-num js-line-number" data-line-number="113"></td>
        <td id="LC113" class="blob-code blob-code-inner js-file-line">    <span class="pl-c1">0xFB</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x40</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>,</td>
      </tr>
      <tr>
        <td id="L114" class="blob-num js-line-number" data-line-number="114"></td>
        <td id="LC114" class="blob-code blob-code-inner js-file-line">    <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x01</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x01</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x01</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>,</td>
      </tr>
      <tr>
        <td id="L115" class="blob-num js-line-number" data-line-number="115"></td>
        <td id="LC115" class="blob-code blob-code-inner js-file-line">    <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0xFA</span>, <span class="pl-c1">0x36</span>, <span class="pl-c1">0xFF</span>, <span class="pl-c1">0xBC</span>, <span class="pl-c1">0x30</span>, <span class="pl-c1">0x8E</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x05</span>, <span class="pl-c1">0xFB</span>, <span class="pl-c1">0xF0</span>, <span class="pl-c1">0xFF</span>, <span class="pl-c1">0xD9</span>, <span class="pl-c1">0x5B</span>, <span class="pl-c1">0xC8</span>,</td>
      </tr>
      <tr>
        <td id="L116" class="blob-num js-line-number" data-line-number="116"></td>
        <td id="LC116" class="blob-code blob-code-inner js-file-line">    <span class="pl-c1">0xFF</span>, <span class="pl-c1">0xD0</span>, <span class="pl-c1">0x9A</span>, <span class="pl-c1">0xBE</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x10</span>, <span class="pl-c1">0xA9</span>, <span class="pl-c1">0xFF</span>, <span class="pl-c1">0xF4</span>, <span class="pl-c1">0x1E</span>, <span class="pl-c1">0xB2</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0xCE</span>, <span class="pl-c1">0xBB</span>, <span class="pl-c1">0xF7</span>,</td>
      </tr>
      <tr>
        <td id="L117" class="blob-num js-line-number" data-line-number="117"></td>
        <td id="LC117" class="blob-code blob-code-inner js-file-line">    <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x01</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x04</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x02</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x02</span>, <span class="pl-c1">0x02</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x0C</span>,</td>
      </tr>
      <tr>
        <td id="L118" class="blob-num js-line-number" data-line-number="118"></td>
        <td id="LC118" class="blob-code blob-code-inner js-file-line">    <span class="pl-c1">0xFF</span>, <span class="pl-c1">0xC2</span>, <span class="pl-c1">0x80</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x01</span>, <span class="pl-c1">0x80</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0xCF</span>, <span class="pl-c1">0x80</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x40</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>,</td>
      </tr>
      <tr>
        <td id="L119" class="blob-num js-line-number" data-line-number="119"></td>
        <td id="LC119" class="blob-code blob-code-inner js-file-line">    <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x01</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x06</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x14</span>,</td>
      </tr>
      <tr>
        <td id="L120" class="blob-num js-line-number" data-line-number="120"></td>
        <td id="LC120" class="blob-code blob-code-inner js-file-line">    <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>,</td>
      </tr>
      <tr>
        <td id="L121" class="blob-num js-line-number" data-line-number="121"></td>
        <td id="LC121" class="blob-code blob-code-inner js-file-line">    <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>,</td>
      </tr>
      <tr>
        <td id="L122" class="blob-num js-line-number" data-line-number="122"></td>
        <td id="LC122" class="blob-code blob-code-inner js-file-line">    <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>,</td>
      </tr>
      <tr>
        <td id="L123" class="blob-num js-line-number" data-line-number="123"></td>
        <td id="LC123" class="blob-code blob-code-inner js-file-line">    <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x03</span>, <span class="pl-c1">0x3F</span>, <span class="pl-c1">0x68</span>, <span class="pl-c1">0xB6</span>, <span class="pl-c1">0x79</span>, <span class="pl-c1">0x35</span>, <span class="pl-c1">0x28</span>, <span class="pl-c1">0xBC</span>, <span class="pl-c1">0xC6</span>, <span class="pl-c1">0x7E</span>, <span class="pl-c1">0xD1</span>, <span class="pl-c1">0x6C</span>,</td>
      </tr>
      <tr>
        <td id="L124" class="blob-num js-line-number" data-line-number="124"></td>
        <td id="LC124" class="blob-code blob-code-inner js-file-line">    <span class="pl-c1">0x80</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x40</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0xB2</span>, <span class="pl-c1">0x6A</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>,</td>
      </tr>
      <tr>
        <td id="L125" class="blob-num js-line-number" data-line-number="125"></td>
        <td id="LC125" class="blob-code blob-code-inner js-file-line">    <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x3F</span>, <span class="pl-c1">0xF0</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x30</span>,</td>
      </tr>
      <tr>
        <td id="L126" class="blob-num js-line-number" data-line-number="126"></td>
        <td id="LC126" class="blob-code blob-code-inner js-file-line">    <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>,</td>
      </tr>
      <tr>
        <td id="L127" class="blob-num js-line-number" data-line-number="127"></td>
        <td id="LC127" class="blob-code blob-code-inner js-file-line">    <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>,</td>
      </tr>
      <tr>
        <td id="L128" class="blob-num js-line-number" data-line-number="128"></td>
        <td id="LC128" class="blob-code blob-code-inner js-file-line">    <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x25</span>, <span class="pl-c1">0x4D</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x2F</span>, <span class="pl-c1">0x70</span>, <span class="pl-c1">0x6D</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x05</span>, <span class="pl-c1">0xAE</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x0C</span>, <span class="pl-c1">0x02</span>, <span class="pl-c1">0xD0</span>,</td>
      </tr>
      <tr>
        <td id="L129" class="blob-num js-line-number" data-line-number="129"></td>
        <td id="LC129" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L130" class="blob-num js-line-number" data-line-number="130"></td>
        <td id="LC130" class="blob-code blob-code-inner js-file-line">    <span class="pl-c"><span class="pl-c">//</span> bank 2, 256 bytes</span></td>
      </tr>
      <tr>
        <td id="L131" class="blob-num js-line-number" data-line-number="131"></td>
        <td id="LC131" class="blob-code blob-code-inner js-file-line">    <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x65</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x54</span>, <span class="pl-c1">0xFF</span>, <span class="pl-c1">0xEF</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>,</td>
      </tr>
      <tr>
        <td id="L132" class="blob-num js-line-number" data-line-number="132"></td>
        <td id="LC132" class="blob-code blob-code-inner js-file-line">    <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x01</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x44</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x0C</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x01</span>, <span class="pl-c1">0x00</span>,</td>
      </tr>
      <tr>
        <td id="L133" class="blob-num js-line-number" data-line-number="133"></td>
        <td id="LC133" class="blob-code blob-code-inner js-file-line">    <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x65</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x54</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0xFF</span>, <span class="pl-c1">0xEF</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>,</td>
      </tr>
      <tr>
        <td id="L134" class="blob-num js-line-number" data-line-number="134"></td>
        <td id="LC134" class="blob-code blob-code-inner js-file-line">    <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>,</td>
      </tr>
      <tr>
        <td id="L135" class="blob-num js-line-number" data-line-number="135"></td>
        <td id="LC135" class="blob-code blob-code-inner js-file-line">    <span class="pl-c1">0x40</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>,</td>
      </tr>
      <tr>
        <td id="L136" class="blob-num js-line-number" data-line-number="136"></td>
        <td id="LC136" class="blob-code blob-code-inner js-file-line">    <span class="pl-c1">0x40</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>,</td>
      </tr>
      <tr>
        <td id="L137" class="blob-num js-line-number" data-line-number="137"></td>
        <td id="LC137" class="blob-code blob-code-inner js-file-line">    <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x01</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x02</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>,</td>
      </tr>
      <tr>
        <td id="L138" class="blob-num js-line-number" data-line-number="138"></td>
        <td id="LC138" class="blob-code blob-code-inner js-file-line">    <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>,</td>
      </tr>
      <tr>
        <td id="L139" class="blob-num js-line-number" data-line-number="139"></td>
        <td id="LC139" class="blob-code blob-code-inner js-file-line">    <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>,</td>
      </tr>
      <tr>
        <td id="L140" class="blob-num js-line-number" data-line-number="140"></td>
        <td id="LC140" class="blob-code blob-code-inner js-file-line">    <span class="pl-c1">0x00</span>, <span class="pl-c1">0x1B</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>,</td>
      </tr>
      <tr>
        <td id="L141" class="blob-num js-line-number" data-line-number="141"></td>
        <td id="LC141" class="blob-code blob-code-inner js-file-line">    <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>,</td>
      </tr>
      <tr>
        <td id="L142" class="blob-num js-line-number" data-line-number="142"></td>
        <td id="LC142" class="blob-code blob-code-inner js-file-line">    <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x40</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>,</td>
      </tr>
      <tr>
        <td id="L143" class="blob-num js-line-number" data-line-number="143"></td>
        <td id="LC143" class="blob-code blob-code-inner js-file-line">    <span class="pl-c1">0x00</span>, <span class="pl-c1">0x1B</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>,</td>
      </tr>
      <tr>
        <td id="L144" class="blob-num js-line-number" data-line-number="144"></td>
        <td id="LC144" class="blob-code blob-code-inner js-file-line">    <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>,</td>
      </tr>
      <tr>
        <td id="L145" class="blob-num js-line-number" data-line-number="145"></td>
        <td id="LC145" class="blob-code blob-code-inner js-file-line">    <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>,</td>
      </tr>
      <tr>
        <td id="L146" class="blob-num js-line-number" data-line-number="146"></td>
        <td id="LC146" class="blob-code blob-code-inner js-file-line">    <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>,</td>
      </tr>
      <tr>
        <td id="L147" class="blob-num js-line-number" data-line-number="147"></td>
        <td id="LC147" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L148" class="blob-num js-line-number" data-line-number="148"></td>
        <td id="LC148" class="blob-code blob-code-inner js-file-line">    <span class="pl-c"><span class="pl-c">//</span> bank 3, 256 bytes</span></td>
      </tr>
      <tr>
        <td id="L149" class="blob-num js-line-number" data-line-number="149"></td>
        <td id="LC149" class="blob-code blob-code-inner js-file-line">    <span class="pl-c1">0xD8</span>, <span class="pl-c1">0xDC</span>, <span class="pl-c1">0xBA</span>, <span class="pl-c1">0xA2</span>, <span class="pl-c1">0xF1</span>, <span class="pl-c1">0xDE</span>, <span class="pl-c1">0xB2</span>, <span class="pl-c1">0xB8</span>, <span class="pl-c1">0xB4</span>, <span class="pl-c1">0xA8</span>, <span class="pl-c1">0x81</span>, <span class="pl-c1">0x91</span>, <span class="pl-c1">0xF7</span>, <span class="pl-c1">0x4A</span>, <span class="pl-c1">0x90</span>, <span class="pl-c1">0x7F</span>,</td>
      </tr>
      <tr>
        <td id="L150" class="blob-num js-line-number" data-line-number="150"></td>
        <td id="LC150" class="blob-code blob-code-inner js-file-line">    <span class="pl-c1">0x91</span>, <span class="pl-c1">0x6A</span>, <span class="pl-c1">0xF3</span>, <span class="pl-c1">0xF9</span>, <span class="pl-c1">0xDB</span>, <span class="pl-c1">0xA8</span>, <span class="pl-c1">0xF9</span>, <span class="pl-c1">0xB0</span>, <span class="pl-c1">0xBA</span>, <span class="pl-c1">0xA0</span>, <span class="pl-c1">0x80</span>, <span class="pl-c1">0xF2</span>, <span class="pl-c1">0xCE</span>, <span class="pl-c1">0x81</span>, <span class="pl-c1">0xF3</span>, <span class="pl-c1">0xC2</span>,</td>
      </tr>
      <tr>
        <td id="L151" class="blob-num js-line-number" data-line-number="151"></td>
        <td id="LC151" class="blob-code blob-code-inner js-file-line">    <span class="pl-c1">0xF1</span>, <span class="pl-c1">0xC1</span>, <span class="pl-c1">0xF2</span>, <span class="pl-c1">0xC3</span>, <span class="pl-c1">0xF3</span>, <span class="pl-c1">0xCC</span>, <span class="pl-c1">0xA2</span>, <span class="pl-c1">0xB2</span>, <span class="pl-c1">0x80</span>, <span class="pl-c1">0xF1</span>, <span class="pl-c1">0xC6</span>, <span class="pl-c1">0xD8</span>, <span class="pl-c1">0x80</span>, <span class="pl-c1">0xBA</span>, <span class="pl-c1">0xA7</span>, <span class="pl-c1">0xDF</span>,</td>
      </tr>
      <tr>
        <td id="L152" class="blob-num js-line-number" data-line-number="152"></td>
        <td id="LC152" class="blob-code blob-code-inner js-file-line">    <span class="pl-c1">0xDF</span>, <span class="pl-c1">0xDF</span>, <span class="pl-c1">0xF2</span>, <span class="pl-c1">0xA7</span>, <span class="pl-c1">0xC3</span>, <span class="pl-c1">0xCB</span>, <span class="pl-c1">0xC5</span>, <span class="pl-c1">0xB6</span>, <span class="pl-c1">0xF0</span>, <span class="pl-c1">0x87</span>, <span class="pl-c1">0xA2</span>, <span class="pl-c1">0x94</span>, <span class="pl-c1">0x24</span>, <span class="pl-c1">0x48</span>, <span class="pl-c1">0x70</span>, <span class="pl-c1">0x3C</span>,</td>
      </tr>
      <tr>
        <td id="L153" class="blob-num js-line-number" data-line-number="153"></td>
        <td id="LC153" class="blob-code blob-code-inner js-file-line">    <span class="pl-c1">0x95</span>, <span class="pl-c1">0x40</span>, <span class="pl-c1">0x68</span>, <span class="pl-c1">0x34</span>, <span class="pl-c1">0x58</span>, <span class="pl-c1">0x9B</span>, <span class="pl-c1">0x78</span>, <span class="pl-c1">0xA2</span>, <span class="pl-c1">0xF1</span>, <span class="pl-c1">0x83</span>, <span class="pl-c1">0x92</span>, <span class="pl-c1">0x2D</span>, <span class="pl-c1">0x55</span>, <span class="pl-c1">0x7D</span>, <span class="pl-c1">0xD8</span>, <span class="pl-c1">0xB1</span>,</td>
      </tr>
      <tr>
        <td id="L154" class="blob-num js-line-number" data-line-number="154"></td>
        <td id="LC154" class="blob-code blob-code-inner js-file-line">    <span class="pl-c1">0xB4</span>, <span class="pl-c1">0xB8</span>, <span class="pl-c1">0xA1</span>, <span class="pl-c1">0xD0</span>, <span class="pl-c1">0x91</span>, <span class="pl-c1">0x80</span>, <span class="pl-c1">0xF2</span>, <span class="pl-c1">0x70</span>, <span class="pl-c1">0xF3</span>, <span class="pl-c1">0x70</span>, <span class="pl-c1">0xF2</span>, <span class="pl-c1">0x7C</span>, <span class="pl-c1">0x80</span>, <span class="pl-c1">0xA8</span>, <span class="pl-c1">0xF1</span>, <span class="pl-c1">0x01</span>,</td>
      </tr>
      <tr>
        <td id="L155" class="blob-num js-line-number" data-line-number="155"></td>
        <td id="LC155" class="blob-code blob-code-inner js-file-line">    <span class="pl-c1">0xB0</span>, <span class="pl-c1">0x98</span>, <span class="pl-c1">0x87</span>, <span class="pl-c1">0xD9</span>, <span class="pl-c1">0x43</span>, <span class="pl-c1">0xD8</span>, <span class="pl-c1">0x86</span>, <span class="pl-c1">0xC9</span>, <span class="pl-c1">0x88</span>, <span class="pl-c1">0xBA</span>, <span class="pl-c1">0xA1</span>, <span class="pl-c1">0xF2</span>, <span class="pl-c1">0x0E</span>, <span class="pl-c1">0xB8</span>, <span class="pl-c1">0x97</span>, <span class="pl-c1">0x80</span>,</td>
      </tr>
      <tr>
        <td id="L156" class="blob-num js-line-number" data-line-number="156"></td>
        <td id="LC156" class="blob-code blob-code-inner js-file-line">    <span class="pl-c1">0xF1</span>, <span class="pl-c1">0xA9</span>, <span class="pl-c1">0xDF</span>, <span class="pl-c1">0xDF</span>, <span class="pl-c1">0xDF</span>, <span class="pl-c1">0xAA</span>, <span class="pl-c1">0xDF</span>, <span class="pl-c1">0xDF</span>, <span class="pl-c1">0xDF</span>, <span class="pl-c1">0xF2</span>, <span class="pl-c1">0xAA</span>, <span class="pl-c1">0xC5</span>, <span class="pl-c1">0xCD</span>, <span class="pl-c1">0xC7</span>, <span class="pl-c1">0xA9</span>, <span class="pl-c1">0x0C</span>,</td>
      </tr>
      <tr>
        <td id="L157" class="blob-num js-line-number" data-line-number="157"></td>
        <td id="LC157" class="blob-code blob-code-inner js-file-line">    <span class="pl-c1">0xC9</span>, <span class="pl-c1">0x2C</span>, <span class="pl-c1">0x97</span>, <span class="pl-c1">0x97</span>, <span class="pl-c1">0x97</span>, <span class="pl-c1">0x97</span>, <span class="pl-c1">0xF1</span>, <span class="pl-c1">0xA9</span>, <span class="pl-c1">0x89</span>, <span class="pl-c1">0x26</span>, <span class="pl-c1">0x46</span>, <span class="pl-c1">0x66</span>, <span class="pl-c1">0xB0</span>, <span class="pl-c1">0xB4</span>, <span class="pl-c1">0xBA</span>, <span class="pl-c1">0x80</span>,</td>
      </tr>
      <tr>
        <td id="L158" class="blob-num js-line-number" data-line-number="158"></td>
        <td id="LC158" class="blob-code blob-code-inner js-file-line">    <span class="pl-c1">0xAC</span>, <span class="pl-c1">0xDE</span>, <span class="pl-c1">0xF2</span>, <span class="pl-c1">0xCA</span>, <span class="pl-c1">0xF1</span>, <span class="pl-c1">0xB2</span>, <span class="pl-c1">0x8C</span>, <span class="pl-c1">0x02</span>, <span class="pl-c1">0xA9</span>, <span class="pl-c1">0xB6</span>, <span class="pl-c1">0x98</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x89</span>, <span class="pl-c1">0x0E</span>, <span class="pl-c1">0x16</span>, <span class="pl-c1">0x1E</span>,</td>
      </tr>
      <tr>
        <td id="L159" class="blob-num js-line-number" data-line-number="159"></td>
        <td id="LC159" class="blob-code blob-code-inner js-file-line">    <span class="pl-c1">0xB8</span>, <span class="pl-c1">0xA9</span>, <span class="pl-c1">0xB4</span>, <span class="pl-c1">0x99</span>, <span class="pl-c1">0x2C</span>, <span class="pl-c1">0x54</span>, <span class="pl-c1">0x7C</span>, <span class="pl-c1">0xB0</span>, <span class="pl-c1">0x8A</span>, <span class="pl-c1">0xA8</span>, <span class="pl-c1">0x96</span>, <span class="pl-c1">0x36</span>, <span class="pl-c1">0x56</span>, <span class="pl-c1">0x76</span>, <span class="pl-c1">0xF1</span>, <span class="pl-c1">0xB9</span>,</td>
      </tr>
      <tr>
        <td id="L160" class="blob-num js-line-number" data-line-number="160"></td>
        <td id="LC160" class="blob-code blob-code-inner js-file-line">    <span class="pl-c1">0xAF</span>, <span class="pl-c1">0xB4</span>, <span class="pl-c1">0xB0</span>, <span class="pl-c1">0x83</span>, <span class="pl-c1">0xC0</span>, <span class="pl-c1">0xB8</span>, <span class="pl-c1">0xA8</span>, <span class="pl-c1">0x97</span>, <span class="pl-c1">0x11</span>, <span class="pl-c1">0xB1</span>, <span class="pl-c1">0x8F</span>, <span class="pl-c1">0x98</span>, <span class="pl-c1">0xB9</span>, <span class="pl-c1">0xAF</span>, <span class="pl-c1">0xF0</span>, <span class="pl-c1">0x24</span>,</td>
      </tr>
      <tr>
        <td id="L161" class="blob-num js-line-number" data-line-number="161"></td>
        <td id="LC161" class="blob-code blob-code-inner js-file-line">    <span class="pl-c1">0x08</span>, <span class="pl-c1">0x44</span>, <span class="pl-c1">0x10</span>, <span class="pl-c1">0x64</span>, <span class="pl-c1">0x18</span>, <span class="pl-c1">0xF1</span>, <span class="pl-c1">0xA3</span>, <span class="pl-c1">0x29</span>, <span class="pl-c1">0x55</span>, <span class="pl-c1">0x7D</span>, <span class="pl-c1">0xAF</span>, <span class="pl-c1">0x83</span>, <span class="pl-c1">0xB5</span>, <span class="pl-c1">0x93</span>, <span class="pl-c1">0xAF</span>, <span class="pl-c1">0xF0</span>,</td>
      </tr>
      <tr>
        <td id="L162" class="blob-num js-line-number" data-line-number="162"></td>
        <td id="LC162" class="blob-code blob-code-inner js-file-line">    <span class="pl-c1">0x00</span>, <span class="pl-c1">0x28</span>, <span class="pl-c1">0x50</span>, <span class="pl-c1">0xF1</span>, <span class="pl-c1">0xA3</span>, <span class="pl-c1">0x86</span>, <span class="pl-c1">0x9F</span>, <span class="pl-c1">0x61</span>, <span class="pl-c1">0xA6</span>, <span class="pl-c1">0xDA</span>, <span class="pl-c1">0xDE</span>, <span class="pl-c1">0xDF</span>, <span class="pl-c1">0xD9</span>, <span class="pl-c1">0xFA</span>, <span class="pl-c1">0xA3</span>, <span class="pl-c1">0x86</span>,</td>
      </tr>
      <tr>
        <td id="L163" class="blob-num js-line-number" data-line-number="163"></td>
        <td id="LC163" class="blob-code blob-code-inner js-file-line">    <span class="pl-c1">0x96</span>, <span class="pl-c1">0xDB</span>, <span class="pl-c1">0x31</span>, <span class="pl-c1">0xA6</span>, <span class="pl-c1">0xD9</span>, <span class="pl-c1">0xF8</span>, <span class="pl-c1">0xDF</span>, <span class="pl-c1">0xBA</span>, <span class="pl-c1">0xA6</span>, <span class="pl-c1">0x8F</span>, <span class="pl-c1">0xC2</span>, <span class="pl-c1">0xC5</span>, <span class="pl-c1">0xC7</span>, <span class="pl-c1">0xB2</span>, <span class="pl-c1">0x8C</span>, <span class="pl-c1">0xC1</span>,</td>
      </tr>
      <tr>
        <td id="L164" class="blob-num js-line-number" data-line-number="164"></td>
        <td id="LC164" class="blob-code blob-code-inner js-file-line">    <span class="pl-c1">0xB8</span>, <span class="pl-c1">0xA2</span>, <span class="pl-c1">0xDF</span>, <span class="pl-c1">0xDF</span>, <span class="pl-c1">0xDF</span>, <span class="pl-c1">0xA3</span>, <span class="pl-c1">0xDF</span>, <span class="pl-c1">0xDF</span>, <span class="pl-c1">0xDF</span>, <span class="pl-c1">0xD8</span>, <span class="pl-c1">0xD8</span>, <span class="pl-c1">0xF1</span>, <span class="pl-c1">0xB8</span>, <span class="pl-c1">0xA8</span>, <span class="pl-c1">0xB2</span>, <span class="pl-c1">0x86</span>,</td>
      </tr>
      <tr>
        <td id="L165" class="blob-num js-line-number" data-line-number="165"></td>
        <td id="LC165" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L166" class="blob-num js-line-number" data-line-number="166"></td>
        <td id="LC166" class="blob-code blob-code-inner js-file-line">    <span class="pl-c"><span class="pl-c">//</span> bank 4, 256 bytes</span></td>
      </tr>
      <tr>
        <td id="L167" class="blob-num js-line-number" data-line-number="167"></td>
        <td id="LC167" class="blob-code blob-code-inner js-file-line">    <span class="pl-c1">0xB4</span>, <span class="pl-c1">0x98</span>, <span class="pl-c1">0x0D</span>, <span class="pl-c1">0x35</span>, <span class="pl-c1">0x5D</span>, <span class="pl-c1">0xB8</span>, <span class="pl-c1">0xAA</span>, <span class="pl-c1">0x98</span>, <span class="pl-c1">0xB0</span>, <span class="pl-c1">0x87</span>, <span class="pl-c1">0x2D</span>, <span class="pl-c1">0x35</span>, <span class="pl-c1">0x3D</span>, <span class="pl-c1">0xB2</span>, <span class="pl-c1">0xB6</span>, <span class="pl-c1">0xBA</span>,</td>
      </tr>
      <tr>
        <td id="L168" class="blob-num js-line-number" data-line-number="168"></td>
        <td id="LC168" class="blob-code blob-code-inner js-file-line">    <span class="pl-c1">0xAF</span>, <span class="pl-c1">0x8C</span>, <span class="pl-c1">0x96</span>, <span class="pl-c1">0x19</span>, <span class="pl-c1">0x8F</span>, <span class="pl-c1">0x9F</span>, <span class="pl-c1">0xA7</span>, <span class="pl-c1">0x0E</span>, <span class="pl-c1">0x16</span>, <span class="pl-c1">0x1E</span>, <span class="pl-c1">0xB4</span>, <span class="pl-c1">0x9A</span>, <span class="pl-c1">0xB8</span>, <span class="pl-c1">0xAA</span>, <span class="pl-c1">0x87</span>, <span class="pl-c1">0x2C</span>,</td>
      </tr>
      <tr>
        <td id="L169" class="blob-num js-line-number" data-line-number="169"></td>
        <td id="LC169" class="blob-code blob-code-inner js-file-line">    <span class="pl-c1">0x54</span>, <span class="pl-c1">0x7C</span>, <span class="pl-c1">0xB9</span>, <span class="pl-c1">0xA3</span>, <span class="pl-c1">0xDE</span>, <span class="pl-c1">0xDF</span>, <span class="pl-c1">0xDF</span>, <span class="pl-c1">0xA3</span>, <span class="pl-c1">0xB1</span>, <span class="pl-c1">0x80</span>, <span class="pl-c1">0xF2</span>, <span class="pl-c1">0xC4</span>, <span class="pl-c1">0xCD</span>, <span class="pl-c1">0xC9</span>, <span class="pl-c1">0xF1</span>, <span class="pl-c1">0xB8</span>,</td>
      </tr>
      <tr>
        <td id="L170" class="blob-num js-line-number" data-line-number="170"></td>
        <td id="LC170" class="blob-code blob-code-inner js-file-line">    <span class="pl-c1">0xA9</span>, <span class="pl-c1">0xB4</span>, <span class="pl-c1">0x99</span>, <span class="pl-c1">0x83</span>, <span class="pl-c1">0x0D</span>, <span class="pl-c1">0x35</span>, <span class="pl-c1">0x5D</span>, <span class="pl-c1">0x89</span>, <span class="pl-c1">0xB9</span>, <span class="pl-c1">0xA3</span>, <span class="pl-c1">0x2D</span>, <span class="pl-c1">0x55</span>, <span class="pl-c1">0x7D</span>, <span class="pl-c1">0xB5</span>, <span class="pl-c1">0x93</span>, <span class="pl-c1">0xA3</span>,</td>
      </tr>
      <tr>
        <td id="L171" class="blob-num js-line-number" data-line-number="171"></td>
        <td id="LC171" class="blob-code blob-code-inner js-file-line">    <span class="pl-c1">0x0E</span>, <span class="pl-c1">0x16</span>, <span class="pl-c1">0x1E</span>, <span class="pl-c1">0xA9</span>, <span class="pl-c1">0x2C</span>, <span class="pl-c1">0x54</span>, <span class="pl-c1">0x7C</span>, <span class="pl-c1">0xB8</span>, <span class="pl-c1">0xB4</span>, <span class="pl-c1">0xB0</span>, <span class="pl-c1">0xF1</span>, <span class="pl-c1">0x97</span>, <span class="pl-c1">0x83</span>, <span class="pl-c1">0xA8</span>, <span class="pl-c1">0x11</span>, <span class="pl-c1">0x84</span>,</td>
      </tr>
      <tr>
        <td id="L172" class="blob-num js-line-number" data-line-number="172"></td>
        <td id="LC172" class="blob-code blob-code-inner js-file-line">    <span class="pl-c1">0xA5</span>, <span class="pl-c1">0x09</span>, <span class="pl-c1">0x98</span>, <span class="pl-c1">0xA3</span>, <span class="pl-c1">0x83</span>, <span class="pl-c1">0xF0</span>, <span class="pl-c1">0xDA</span>, <span class="pl-c1">0x24</span>, <span class="pl-c1">0x08</span>, <span class="pl-c1">0x44</span>, <span class="pl-c1">0x10</span>, <span class="pl-c1">0x64</span>, <span class="pl-c1">0x18</span>, <span class="pl-c1">0xD8</span>, <span class="pl-c1">0xF1</span>, <span class="pl-c1">0xA5</span>,</td>
      </tr>
      <tr>
        <td id="L173" class="blob-num js-line-number" data-line-number="173"></td>
        <td id="LC173" class="blob-code blob-code-inner js-file-line">    <span class="pl-c1">0x29</span>, <span class="pl-c1">0x55</span>, <span class="pl-c1">0x7D</span>, <span class="pl-c1">0xA5</span>, <span class="pl-c1">0x85</span>, <span class="pl-c1">0x95</span>, <span class="pl-c1">0x02</span>, <span class="pl-c1">0x1A</span>, <span class="pl-c1">0x2E</span>, <span class="pl-c1">0x3A</span>, <span class="pl-c1">0x56</span>, <span class="pl-c1">0x5A</span>, <span class="pl-c1">0x40</span>, <span class="pl-c1">0x48</span>, <span class="pl-c1">0xF9</span>, <span class="pl-c1">0xF3</span>,</td>
      </tr>
      <tr>
        <td id="L174" class="blob-num js-line-number" data-line-number="174"></td>
        <td id="LC174" class="blob-code blob-code-inner js-file-line">    <span class="pl-c1">0xA3</span>, <span class="pl-c1">0xD9</span>, <span class="pl-c1">0xF8</span>, <span class="pl-c1">0xF0</span>, <span class="pl-c1">0x98</span>, <span class="pl-c1">0x83</span>, <span class="pl-c1">0x24</span>, <span class="pl-c1">0x08</span>, <span class="pl-c1">0x44</span>, <span class="pl-c1">0x10</span>, <span class="pl-c1">0x64</span>, <span class="pl-c1">0x18</span>, <span class="pl-c1">0x97</span>, <span class="pl-c1">0x82</span>, <span class="pl-c1">0xA8</span>, <span class="pl-c1">0xF1</span>,</td>
      </tr>
      <tr>
        <td id="L175" class="blob-num js-line-number" data-line-number="175"></td>
        <td id="LC175" class="blob-code blob-code-inner js-file-line">    <span class="pl-c1">0x11</span>, <span class="pl-c1">0xF0</span>, <span class="pl-c1">0x98</span>, <span class="pl-c1">0xA2</span>, <span class="pl-c1">0x24</span>, <span class="pl-c1">0x08</span>, <span class="pl-c1">0x44</span>, <span class="pl-c1">0x10</span>, <span class="pl-c1">0x64</span>, <span class="pl-c1">0x18</span>, <span class="pl-c1">0xDA</span>, <span class="pl-c1">0xF3</span>, <span class="pl-c1">0xDE</span>, <span class="pl-c1">0xD8</span>, <span class="pl-c1">0x83</span>, <span class="pl-c1">0xA5</span>,</td>
      </tr>
      <tr>
        <td id="L176" class="blob-num js-line-number" data-line-number="176"></td>
        <td id="LC176" class="blob-code blob-code-inner js-file-line">    <span class="pl-c1">0x94</span>, <span class="pl-c1">0x01</span>, <span class="pl-c1">0xD9</span>, <span class="pl-c1">0xA3</span>, <span class="pl-c1">0x02</span>, <span class="pl-c1">0xF1</span>, <span class="pl-c1">0xA2</span>, <span class="pl-c1">0xC3</span>, <span class="pl-c1">0xC5</span>, <span class="pl-c1">0xC7</span>, <span class="pl-c1">0xD8</span>, <span class="pl-c1">0xF1</span>, <span class="pl-c1">0x84</span>, <span class="pl-c1">0x92</span>, <span class="pl-c1">0xA2</span>, <span class="pl-c1">0x4D</span>,</td>
      </tr>
      <tr>
        <td id="L177" class="blob-num js-line-number" data-line-number="177"></td>
        <td id="LC177" class="blob-code blob-code-inner js-file-line">    <span class="pl-c1">0xDA</span>, <span class="pl-c1">0x2A</span>, <span class="pl-c1">0xD8</span>, <span class="pl-c1">0x48</span>, <span class="pl-c1">0x69</span>, <span class="pl-c1">0xD9</span>, <span class="pl-c1">0x2A</span>, <span class="pl-c1">0xD8</span>, <span class="pl-c1">0x68</span>, <span class="pl-c1">0x55</span>, <span class="pl-c1">0xDA</span>, <span class="pl-c1">0x32</span>, <span class="pl-c1">0xD8</span>, <span class="pl-c1">0x50</span>, <span class="pl-c1">0x71</span>, <span class="pl-c1">0xD9</span>,</td>
      </tr>
      <tr>
        <td id="L178" class="blob-num js-line-number" data-line-number="178"></td>
        <td id="LC178" class="blob-code blob-code-inner js-file-line">    <span class="pl-c1">0x32</span>, <span class="pl-c1">0xD8</span>, <span class="pl-c1">0x70</span>, <span class="pl-c1">0x5D</span>, <span class="pl-c1">0xDA</span>, <span class="pl-c1">0x3A</span>, <span class="pl-c1">0xD8</span>, <span class="pl-c1">0x58</span>, <span class="pl-c1">0x79</span>, <span class="pl-c1">0xD9</span>, <span class="pl-c1">0x3A</span>, <span class="pl-c1">0xD8</span>, <span class="pl-c1">0x78</span>, <span class="pl-c1">0x93</span>, <span class="pl-c1">0xA3</span>, <span class="pl-c1">0x4D</span>,</td>
      </tr>
      <tr>
        <td id="L179" class="blob-num js-line-number" data-line-number="179"></td>
        <td id="LC179" class="blob-code blob-code-inner js-file-line">    <span class="pl-c1">0xDA</span>, <span class="pl-c1">0x2A</span>, <span class="pl-c1">0xD8</span>, <span class="pl-c1">0x48</span>, <span class="pl-c1">0x69</span>, <span class="pl-c1">0xD9</span>, <span class="pl-c1">0x2A</span>, <span class="pl-c1">0xD8</span>, <span class="pl-c1">0x68</span>, <span class="pl-c1">0x55</span>, <span class="pl-c1">0xDA</span>, <span class="pl-c1">0x32</span>, <span class="pl-c1">0xD8</span>, <span class="pl-c1">0x50</span>, <span class="pl-c1">0x71</span>, <span class="pl-c1">0xD9</span>,</td>
      </tr>
      <tr>
        <td id="L180" class="blob-num js-line-number" data-line-number="180"></td>
        <td id="LC180" class="blob-code blob-code-inner js-file-line">    <span class="pl-c1">0x32</span>, <span class="pl-c1">0xD8</span>, <span class="pl-c1">0x70</span>, <span class="pl-c1">0x5D</span>, <span class="pl-c1">0xDA</span>, <span class="pl-c1">0x3A</span>, <span class="pl-c1">0xD8</span>, <span class="pl-c1">0x58</span>, <span class="pl-c1">0x79</span>, <span class="pl-c1">0xD9</span>, <span class="pl-c1">0x3A</span>, <span class="pl-c1">0xD8</span>, <span class="pl-c1">0x78</span>, <span class="pl-c1">0xA8</span>, <span class="pl-c1">0x8A</span>, <span class="pl-c1">0x9A</span>,</td>
      </tr>
      <tr>
        <td id="L181" class="blob-num js-line-number" data-line-number="181"></td>
        <td id="LC181" class="blob-code blob-code-inner js-file-line">    <span class="pl-c1">0xF0</span>, <span class="pl-c1">0x28</span>, <span class="pl-c1">0x50</span>, <span class="pl-c1">0x78</span>, <span class="pl-c1">0x9E</span>, <span class="pl-c1">0xF3</span>, <span class="pl-c1">0x88</span>, <span class="pl-c1">0x18</span>, <span class="pl-c1">0xF1</span>, <span class="pl-c1">0x9F</span>, <span class="pl-c1">0x1D</span>, <span class="pl-c1">0x98</span>, <span class="pl-c1">0xA8</span>, <span class="pl-c1">0xD9</span>, <span class="pl-c1">0x08</span>, <span class="pl-c1">0xD8</span>,</td>
      </tr>
      <tr>
        <td id="L182" class="blob-num js-line-number" data-line-number="182"></td>
        <td id="LC182" class="blob-code blob-code-inner js-file-line">    <span class="pl-c1">0xC8</span>, <span class="pl-c1">0x9F</span>, <span class="pl-c1">0x12</span>, <span class="pl-c1">0x9E</span>, <span class="pl-c1">0xF3</span>, <span class="pl-c1">0x15</span>, <span class="pl-c1">0xA8</span>, <span class="pl-c1">0xDA</span>, <span class="pl-c1">0x12</span>, <span class="pl-c1">0x10</span>, <span class="pl-c1">0xD8</span>, <span class="pl-c1">0xF1</span>, <span class="pl-c1">0xAF</span>, <span class="pl-c1">0xC8</span>, <span class="pl-c1">0x97</span>, <span class="pl-c1">0x87</span>,</td>
      </tr>
      <tr>
        <td id="L183" class="blob-num js-line-number" data-line-number="183"></td>
        <td id="LC183" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L184" class="blob-num js-line-number" data-line-number="184"></td>
        <td id="LC184" class="blob-code blob-code-inner js-file-line">    <span class="pl-c"><span class="pl-c">//</span> bank 5, 256 bytes</span></td>
      </tr>
      <tr>
        <td id="L185" class="blob-num js-line-number" data-line-number="185"></td>
        <td id="LC185" class="blob-code blob-code-inner js-file-line">    <span class="pl-c1">0x34</span>, <span class="pl-c1">0xB5</span>, <span class="pl-c1">0xB9</span>, <span class="pl-c1">0x94</span>, <span class="pl-c1">0xA4</span>, <span class="pl-c1">0x21</span>, <span class="pl-c1">0xF3</span>, <span class="pl-c1">0xD9</span>, <span class="pl-c1">0x22</span>, <span class="pl-c1">0xD8</span>, <span class="pl-c1">0xF2</span>, <span class="pl-c1">0x2D</span>, <span class="pl-c1">0xF3</span>, <span class="pl-c1">0xD9</span>, <span class="pl-c1">0x2A</span>, <span class="pl-c1">0xD8</span>,</td>
      </tr>
      <tr>
        <td id="L186" class="blob-num js-line-number" data-line-number="186"></td>
        <td id="LC186" class="blob-code blob-code-inner js-file-line">    <span class="pl-c1">0xF2</span>, <span class="pl-c1">0x35</span>, <span class="pl-c1">0xF3</span>, <span class="pl-c1">0xD9</span>, <span class="pl-c1">0x32</span>, <span class="pl-c1">0xD8</span>, <span class="pl-c1">0x81</span>, <span class="pl-c1">0xA4</span>, <span class="pl-c1">0x60</span>, <span class="pl-c1">0x60</span>, <span class="pl-c1">0x61</span>, <span class="pl-c1">0xD9</span>, <span class="pl-c1">0x61</span>, <span class="pl-c1">0xD8</span>, <span class="pl-c1">0x6C</span>, <span class="pl-c1">0x68</span>,</td>
      </tr>
      <tr>
        <td id="L187" class="blob-num js-line-number" data-line-number="187"></td>
        <td id="LC187" class="blob-code blob-code-inner js-file-line">    <span class="pl-c1">0x69</span>, <span class="pl-c1">0xD9</span>, <span class="pl-c1">0x69</span>, <span class="pl-c1">0xD8</span>, <span class="pl-c1">0x74</span>, <span class="pl-c1">0x70</span>, <span class="pl-c1">0x71</span>, <span class="pl-c1">0xD9</span>, <span class="pl-c1">0x71</span>, <span class="pl-c1">0xD8</span>, <span class="pl-c1">0xB1</span>, <span class="pl-c1">0xA3</span>, <span class="pl-c1">0x84</span>, <span class="pl-c1">0x19</span>, <span class="pl-c1">0x3D</span>, <span class="pl-c1">0x5D</span>,</td>
      </tr>
      <tr>
        <td id="L188" class="blob-num js-line-number" data-line-number="188"></td>
        <td id="LC188" class="blob-code blob-code-inner js-file-line">    <span class="pl-c1">0xA3</span>, <span class="pl-c1">0x83</span>, <span class="pl-c1">0x1A</span>, <span class="pl-c1">0x3E</span>, <span class="pl-c1">0x5E</span>, <span class="pl-c1">0x93</span>, <span class="pl-c1">0x10</span>, <span class="pl-c1">0x30</span>, <span class="pl-c1">0x81</span>, <span class="pl-c1">0x10</span>, <span class="pl-c1">0x11</span>, <span class="pl-c1">0xB8</span>, <span class="pl-c1">0xB0</span>, <span class="pl-c1">0xAF</span>, <span class="pl-c1">0x8F</span>, <span class="pl-c1">0x94</span>,</td>
      </tr>
      <tr>
        <td id="L189" class="blob-num js-line-number" data-line-number="189"></td>
        <td id="LC189" class="blob-code blob-code-inner js-file-line">    <span class="pl-c1">0xF2</span>, <span class="pl-c1">0xDA</span>, <span class="pl-c1">0x3E</span>, <span class="pl-c1">0xD8</span>, <span class="pl-c1">0xB4</span>, <span class="pl-c1">0x9A</span>, <span class="pl-c1">0xA8</span>, <span class="pl-c1">0x87</span>, <span class="pl-c1">0x29</span>, <span class="pl-c1">0xDA</span>, <span class="pl-c1">0xF8</span>, <span class="pl-c1">0xD8</span>, <span class="pl-c1">0x87</span>, <span class="pl-c1">0x9A</span>, <span class="pl-c1">0x35</span>, <span class="pl-c1">0xDA</span>,</td>
      </tr>
      <tr>
        <td id="L190" class="blob-num js-line-number" data-line-number="190"></td>
        <td id="LC190" class="blob-code blob-code-inner js-file-line">    <span class="pl-c1">0xF8</span>, <span class="pl-c1">0xD8</span>, <span class="pl-c1">0x87</span>, <span class="pl-c1">0x9A</span>, <span class="pl-c1">0x3D</span>, <span class="pl-c1">0xDA</span>, <span class="pl-c1">0xF8</span>, <span class="pl-c1">0xD8</span>, <span class="pl-c1">0xB1</span>, <span class="pl-c1">0xB9</span>, <span class="pl-c1">0xA4</span>, <span class="pl-c1">0x98</span>, <span class="pl-c1">0x85</span>, <span class="pl-c1">0x02</span>, <span class="pl-c1">0x2E</span>, <span class="pl-c1">0x56</span>,</td>
      </tr>
      <tr>
        <td id="L191" class="blob-num js-line-number" data-line-number="191"></td>
        <td id="LC191" class="blob-code blob-code-inner js-file-line">    <span class="pl-c1">0xA5</span>, <span class="pl-c1">0x81</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x0C</span>, <span class="pl-c1">0x14</span>, <span class="pl-c1">0xA3</span>, <span class="pl-c1">0x97</span>, <span class="pl-c1">0xB0</span>, <span class="pl-c1">0x8A</span>, <span class="pl-c1">0xF1</span>, <span class="pl-c1">0x2D</span>, <span class="pl-c1">0xD9</span>, <span class="pl-c1">0x28</span>, <span class="pl-c1">0xD8</span>, <span class="pl-c1">0x4D</span>, <span class="pl-c1">0xD9</span>,</td>
      </tr>
      <tr>
        <td id="L192" class="blob-num js-line-number" data-line-number="192"></td>
        <td id="LC192" class="blob-code blob-code-inner js-file-line">    <span class="pl-c1">0x48</span>, <span class="pl-c1">0xD8</span>, <span class="pl-c1">0x6D</span>, <span class="pl-c1">0xD9</span>, <span class="pl-c1">0x68</span>, <span class="pl-c1">0xD8</span>, <span class="pl-c1">0xB1</span>, <span class="pl-c1">0x84</span>, <span class="pl-c1">0x0D</span>, <span class="pl-c1">0xDA</span>, <span class="pl-c1">0x0E</span>, <span class="pl-c1">0xD8</span>, <span class="pl-c1">0xA3</span>, <span class="pl-c1">0x29</span>, <span class="pl-c1">0x83</span>, <span class="pl-c1">0xDA</span>,</td>
      </tr>
      <tr>
        <td id="L193" class="blob-num js-line-number" data-line-number="193"></td>
        <td id="LC193" class="blob-code blob-code-inner js-file-line">    <span class="pl-c1">0x2C</span>, <span class="pl-c1">0x0E</span>, <span class="pl-c1">0xD8</span>, <span class="pl-c1">0xA3</span>, <span class="pl-c1">0x84</span>, <span class="pl-c1">0x49</span>, <span class="pl-c1">0x83</span>, <span class="pl-c1">0xDA</span>, <span class="pl-c1">0x2C</span>, <span class="pl-c1">0x4C</span>, <span class="pl-c1">0x0E</span>, <span class="pl-c1">0xD8</span>, <span class="pl-c1">0xB8</span>, <span class="pl-c1">0xB0</span>, <span class="pl-c1">0xA8</span>, <span class="pl-c1">0x8A</span>,</td>
      </tr>
      <tr>
        <td id="L194" class="blob-num js-line-number" data-line-number="194"></td>
        <td id="LC194" class="blob-code blob-code-inner js-file-line">    <span class="pl-c1">0x9A</span>, <span class="pl-c1">0xF5</span>, <span class="pl-c1">0x20</span>, <span class="pl-c1">0xAA</span>, <span class="pl-c1">0xDA</span>, <span class="pl-c1">0xDF</span>, <span class="pl-c1">0xD8</span>, <span class="pl-c1">0xA8</span>, <span class="pl-c1">0x40</span>, <span class="pl-c1">0xAA</span>, <span class="pl-c1">0xD0</span>, <span class="pl-c1">0xDA</span>, <span class="pl-c1">0xDE</span>, <span class="pl-c1">0xD8</span>, <span class="pl-c1">0xA8</span>, <span class="pl-c1">0x60</span>,</td>
      </tr>
      <tr>
        <td id="L195" class="blob-num js-line-number" data-line-number="195"></td>
        <td id="LC195" class="blob-code blob-code-inner js-file-line">    <span class="pl-c1">0xAA</span>, <span class="pl-c1">0xDA</span>, <span class="pl-c1">0xD0</span>, <span class="pl-c1">0xDF</span>, <span class="pl-c1">0xD8</span>, <span class="pl-c1">0xF1</span>, <span class="pl-c1">0x97</span>, <span class="pl-c1">0x86</span>, <span class="pl-c1">0xA8</span>, <span class="pl-c1">0x31</span>, <span class="pl-c1">0x9B</span>, <span class="pl-c1">0x06</span>, <span class="pl-c1">0x99</span>, <span class="pl-c1">0x07</span>, <span class="pl-c1">0xAB</span>, <span class="pl-c1">0x97</span>,</td>
      </tr>
      <tr>
        <td id="L196" class="blob-num js-line-number" data-line-number="196"></td>
        <td id="LC196" class="blob-code blob-code-inner js-file-line">    <span class="pl-c1">0x28</span>, <span class="pl-c1">0x88</span>, <span class="pl-c1">0x9B</span>, <span class="pl-c1">0xF0</span>, <span class="pl-c1">0x0C</span>, <span class="pl-c1">0x20</span>, <span class="pl-c1">0x14</span>, <span class="pl-c1">0x40</span>, <span class="pl-c1">0xB8</span>, <span class="pl-c1">0xB0</span>, <span class="pl-c1">0xB4</span>, <span class="pl-c1">0xA8</span>, <span class="pl-c1">0x8C</span>, <span class="pl-c1">0x9C</span>, <span class="pl-c1">0xF0</span>, <span class="pl-c1">0x04</span>,</td>
      </tr>
      <tr>
        <td id="L197" class="blob-num js-line-number" data-line-number="197"></td>
        <td id="LC197" class="blob-code blob-code-inner js-file-line">    <span class="pl-c1">0x28</span>, <span class="pl-c1">0x51</span>, <span class="pl-c1">0x79</span>, <span class="pl-c1">0x1D</span>, <span class="pl-c1">0x30</span>, <span class="pl-c1">0x14</span>, <span class="pl-c1">0x38</span>, <span class="pl-c1">0xB2</span>, <span class="pl-c1">0x82</span>, <span class="pl-c1">0xAB</span>, <span class="pl-c1">0xD0</span>, <span class="pl-c1">0x98</span>, <span class="pl-c1">0x2C</span>, <span class="pl-c1">0x50</span>, <span class="pl-c1">0x50</span>, <span class="pl-c1">0x78</span>,</td>
      </tr>
      <tr>
        <td id="L198" class="blob-num js-line-number" data-line-number="198"></td>
        <td id="LC198" class="blob-code blob-code-inner js-file-line">    <span class="pl-c1">0x78</span>, <span class="pl-c1">0x9B</span>, <span class="pl-c1">0xF1</span>, <span class="pl-c1">0x1A</span>, <span class="pl-c1">0xB0</span>, <span class="pl-c1">0xF0</span>, <span class="pl-c1">0x8A</span>, <span class="pl-c1">0x9C</span>, <span class="pl-c1">0xA8</span>, <span class="pl-c1">0x29</span>, <span class="pl-c1">0x51</span>, <span class="pl-c1">0x79</span>, <span class="pl-c1">0x8B</span>, <span class="pl-c1">0x29</span>, <span class="pl-c1">0x51</span>, <span class="pl-c1">0x79</span>,</td>
      </tr>
      <tr>
        <td id="L199" class="blob-num js-line-number" data-line-number="199"></td>
        <td id="LC199" class="blob-code blob-code-inner js-file-line">    <span class="pl-c1">0x8A</span>, <span class="pl-c1">0x24</span>, <span class="pl-c1">0x70</span>, <span class="pl-c1">0x59</span>, <span class="pl-c1">0x8B</span>, <span class="pl-c1">0x20</span>, <span class="pl-c1">0x58</span>, <span class="pl-c1">0x71</span>, <span class="pl-c1">0x8A</span>, <span class="pl-c1">0x44</span>, <span class="pl-c1">0x69</span>, <span class="pl-c1">0x38</span>, <span class="pl-c1">0x8B</span>, <span class="pl-c1">0x39</span>, <span class="pl-c1">0x40</span>, <span class="pl-c1">0x68</span>,</td>
      </tr>
      <tr>
        <td id="L200" class="blob-num js-line-number" data-line-number="200"></td>
        <td id="LC200" class="blob-code blob-code-inner js-file-line">    <span class="pl-c1">0x8A</span>, <span class="pl-c1">0x64</span>, <span class="pl-c1">0x48</span>, <span class="pl-c1">0x31</span>, <span class="pl-c1">0x8B</span>, <span class="pl-c1">0x30</span>, <span class="pl-c1">0x49</span>, <span class="pl-c1">0x60</span>, <span class="pl-c1">0xA5</span>, <span class="pl-c1">0x88</span>, <span class="pl-c1">0x20</span>, <span class="pl-c1">0x09</span>, <span class="pl-c1">0x71</span>, <span class="pl-c1">0x58</span>, <span class="pl-c1">0x44</span>, <span class="pl-c1">0x68</span>,</td>
      </tr>
      <tr>
        <td id="L201" class="blob-num js-line-number" data-line-number="201"></td>
        <td id="LC201" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L202" class="blob-num js-line-number" data-line-number="202"></td>
        <td id="LC202" class="blob-code blob-code-inner js-file-line">    <span class="pl-c"><span class="pl-c">//</span> bank 6, 256 bytes</span></td>
      </tr>
      <tr>
        <td id="L203" class="blob-num js-line-number" data-line-number="203"></td>
        <td id="LC203" class="blob-code blob-code-inner js-file-line">    <span class="pl-c1">0x11</span>, <span class="pl-c1">0x39</span>, <span class="pl-c1">0x64</span>, <span class="pl-c1">0x49</span>, <span class="pl-c1">0x30</span>, <span class="pl-c1">0x19</span>, <span class="pl-c1">0xF1</span>, <span class="pl-c1">0xAC</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x2C</span>, <span class="pl-c1">0x54</span>, <span class="pl-c1">0x7C</span>, <span class="pl-c1">0xF0</span>, <span class="pl-c1">0x8C</span>, <span class="pl-c1">0xA8</span>, <span class="pl-c1">0x04</span>,</td>
      </tr>
      <tr>
        <td id="L204" class="blob-num js-line-number" data-line-number="204"></td>
        <td id="LC204" class="blob-code blob-code-inner js-file-line">    <span class="pl-c1">0x28</span>, <span class="pl-c1">0x50</span>, <span class="pl-c1">0x78</span>, <span class="pl-c1">0xF1</span>, <span class="pl-c1">0x88</span>, <span class="pl-c1">0x97</span>, <span class="pl-c1">0x26</span>, <span class="pl-c1">0xA8</span>, <span class="pl-c1">0x59</span>, <span class="pl-c1">0x98</span>, <span class="pl-c1">0xAC</span>, <span class="pl-c1">0x8C</span>, <span class="pl-c1">0x02</span>, <span class="pl-c1">0x26</span>, <span class="pl-c1">0x46</span>, <span class="pl-c1">0x66</span>,</td>
      </tr>
      <tr>
        <td id="L205" class="blob-num js-line-number" data-line-number="205"></td>
        <td id="LC205" class="blob-code blob-code-inner js-file-line">    <span class="pl-c1">0xF0</span>, <span class="pl-c1">0x89</span>, <span class="pl-c1">0x9C</span>, <span class="pl-c1">0xA8</span>, <span class="pl-c1">0x29</span>, <span class="pl-c1">0x51</span>, <span class="pl-c1">0x79</span>, <span class="pl-c1">0x24</span>, <span class="pl-c1">0x70</span>, <span class="pl-c1">0x59</span>, <span class="pl-c1">0x44</span>, <span class="pl-c1">0x69</span>, <span class="pl-c1">0x38</span>, <span class="pl-c1">0x64</span>, <span class="pl-c1">0x48</span>, <span class="pl-c1">0x31</span>,</td>
      </tr>
      <tr>
        <td id="L206" class="blob-num js-line-number" data-line-number="206"></td>
        <td id="LC206" class="blob-code blob-code-inner js-file-line">    <span class="pl-c1">0xA9</span>, <span class="pl-c1">0x88</span>, <span class="pl-c1">0x09</span>, <span class="pl-c1">0x20</span>, <span class="pl-c1">0x59</span>, <span class="pl-c1">0x70</span>, <span class="pl-c1">0xAB</span>, <span class="pl-c1">0x11</span>, <span class="pl-c1">0x38</span>, <span class="pl-c1">0x40</span>, <span class="pl-c1">0x69</span>, <span class="pl-c1">0xA8</span>, <span class="pl-c1">0x19</span>, <span class="pl-c1">0x31</span>, <span class="pl-c1">0x48</span>, <span class="pl-c1">0x60</span>,</td>
      </tr>
      <tr>
        <td id="L207" class="blob-num js-line-number" data-line-number="207"></td>
        <td id="LC207" class="blob-code blob-code-inner js-file-line">    <span class="pl-c1">0x8C</span>, <span class="pl-c1">0xA8</span>, <span class="pl-c1">0x3C</span>, <span class="pl-c1">0x41</span>, <span class="pl-c1">0x5C</span>, <span class="pl-c1">0x20</span>, <span class="pl-c1">0x7C</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0xF1</span>, <span class="pl-c1">0x87</span>, <span class="pl-c1">0x98</span>, <span class="pl-c1">0x19</span>, <span class="pl-c1">0x86</span>, <span class="pl-c1">0xA8</span>, <span class="pl-c1">0x6E</span>, <span class="pl-c1">0x76</span>,</td>
      </tr>
      <tr>
        <td id="L208" class="blob-num js-line-number" data-line-number="208"></td>
        <td id="LC208" class="blob-code blob-code-inner js-file-line">    <span class="pl-c1">0x7E</span>, <span class="pl-c1">0xA9</span>, <span class="pl-c1">0x99</span>, <span class="pl-c1">0x88</span>, <span class="pl-c1">0x2D</span>, <span class="pl-c1">0x55</span>, <span class="pl-c1">0x7D</span>, <span class="pl-c1">0x9E</span>, <span class="pl-c1">0xB9</span>, <span class="pl-c1">0xA3</span>, <span class="pl-c1">0x8A</span>, <span class="pl-c1">0x22</span>, <span class="pl-c1">0x8A</span>, <span class="pl-c1">0x6E</span>, <span class="pl-c1">0x8A</span>, <span class="pl-c1">0x56</span>,</td>
      </tr>
      <tr>
        <td id="L209" class="blob-num js-line-number" data-line-number="209"></td>
        <td id="LC209" class="blob-code blob-code-inner js-file-line">    <span class="pl-c1">0x8A</span>, <span class="pl-c1">0x5E</span>, <span class="pl-c1">0x9F</span>, <span class="pl-c1">0xB1</span>, <span class="pl-c1">0x83</span>, <span class="pl-c1">0x06</span>, <span class="pl-c1">0x26</span>, <span class="pl-c1">0x46</span>, <span class="pl-c1">0x66</span>, <span class="pl-c1">0x0E</span>, <span class="pl-c1">0x2E</span>, <span class="pl-c1">0x4E</span>, <span class="pl-c1">0x6E</span>, <span class="pl-c1">0x9D</span>, <span class="pl-c1">0xB8</span>, <span class="pl-c1">0xAD</span>,</td>
      </tr>
      <tr>
        <td id="L210" class="blob-num js-line-number" data-line-number="210"></td>
        <td id="LC210" class="blob-code blob-code-inner js-file-line">    <span class="pl-c1">0x00</span>, <span class="pl-c1">0x2C</span>, <span class="pl-c1">0x54</span>, <span class="pl-c1">0x7C</span>, <span class="pl-c1">0xF2</span>, <span class="pl-c1">0xB1</span>, <span class="pl-c1">0x8C</span>, <span class="pl-c1">0xB4</span>, <span class="pl-c1">0x99</span>, <span class="pl-c1">0xB9</span>, <span class="pl-c1">0xA3</span>, <span class="pl-c1">0x2D</span>, <span class="pl-c1">0x55</span>, <span class="pl-c1">0x7D</span>, <span class="pl-c1">0x81</span>, <span class="pl-c1">0x91</span>,</td>
      </tr>
      <tr>
        <td id="L211" class="blob-num js-line-number" data-line-number="211"></td>
        <td id="LC211" class="blob-code blob-code-inner js-file-line">    <span class="pl-c1">0xAC</span>, <span class="pl-c1">0x38</span>, <span class="pl-c1">0xAD</span>, <span class="pl-c1">0x3A</span>, <span class="pl-c1">0xB5</span>, <span class="pl-c1">0x83</span>, <span class="pl-c1">0x91</span>, <span class="pl-c1">0xAC</span>, <span class="pl-c1">0x2D</span>, <span class="pl-c1">0xD9</span>, <span class="pl-c1">0x28</span>, <span class="pl-c1">0xD8</span>, <span class="pl-c1">0x4D</span>, <span class="pl-c1">0xD9</span>, <span class="pl-c1">0x48</span>, <span class="pl-c1">0xD8</span>,</td>
      </tr>
      <tr>
        <td id="L212" class="blob-num js-line-number" data-line-number="212"></td>
        <td id="LC212" class="blob-code blob-code-inner js-file-line">    <span class="pl-c1">0x6D</span>, <span class="pl-c1">0xD9</span>, <span class="pl-c1">0x68</span>, <span class="pl-c1">0xD8</span>, <span class="pl-c1">0x8C</span>, <span class="pl-c1">0x9D</span>, <span class="pl-c1">0xAE</span>, <span class="pl-c1">0x29</span>, <span class="pl-c1">0xD9</span>, <span class="pl-c1">0x04</span>, <span class="pl-c1">0xAE</span>, <span class="pl-c1">0xD8</span>, <span class="pl-c1">0x51</span>, <span class="pl-c1">0xD9</span>, <span class="pl-c1">0x04</span>, <span class="pl-c1">0xAE</span>,</td>
      </tr>
      <tr>
        <td id="L213" class="blob-num js-line-number" data-line-number="213"></td>
        <td id="LC213" class="blob-code blob-code-inner js-file-line">    <span class="pl-c1">0xD8</span>, <span class="pl-c1">0x79</span>, <span class="pl-c1">0xD9</span>, <span class="pl-c1">0x04</span>, <span class="pl-c1">0xD8</span>, <span class="pl-c1">0x81</span>, <span class="pl-c1">0xF3</span>, <span class="pl-c1">0x9D</span>, <span class="pl-c1">0xAD</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x8D</span>, <span class="pl-c1">0xAE</span>, <span class="pl-c1">0x19</span>, <span class="pl-c1">0x81</span>, <span class="pl-c1">0xAD</span>, <span class="pl-c1">0xD9</span>,</td>
      </tr>
      <tr>
        <td id="L214" class="blob-num js-line-number" data-line-number="214"></td>
        <td id="LC214" class="blob-code blob-code-inner js-file-line">    <span class="pl-c1">0x01</span>, <span class="pl-c1">0xD8</span>, <span class="pl-c1">0xF2</span>, <span class="pl-c1">0xAE</span>, <span class="pl-c1">0xDA</span>, <span class="pl-c1">0x26</span>, <span class="pl-c1">0xD8</span>, <span class="pl-c1">0x8E</span>, <span class="pl-c1">0x91</span>, <span class="pl-c1">0x29</span>, <span class="pl-c1">0x83</span>, <span class="pl-c1">0xA7</span>, <span class="pl-c1">0xD9</span>, <span class="pl-c1">0xAD</span>, <span class="pl-c1">0xAD</span>, <span class="pl-c1">0xAD</span>,</td>
      </tr>
      <tr>
        <td id="L215" class="blob-num js-line-number" data-line-number="215"></td>
        <td id="LC215" class="blob-code blob-code-inner js-file-line">    <span class="pl-c1">0xAD</span>, <span class="pl-c1">0xF3</span>, <span class="pl-c1">0x2A</span>, <span class="pl-c1">0xD8</span>, <span class="pl-c1">0xD8</span>, <span class="pl-c1">0xF1</span>, <span class="pl-c1">0xB0</span>, <span class="pl-c1">0xAC</span>, <span class="pl-c1">0x89</span>, <span class="pl-c1">0x91</span>, <span class="pl-c1">0x3E</span>, <span class="pl-c1">0x5E</span>, <span class="pl-c1">0x76</span>, <span class="pl-c1">0xF3</span>, <span class="pl-c1">0xAC</span>, <span class="pl-c1">0x2E</span>,</td>
      </tr>
      <tr>
        <td id="L216" class="blob-num js-line-number" data-line-number="216"></td>
        <td id="LC216" class="blob-code blob-code-inner js-file-line">    <span class="pl-c1">0x2E</span>, <span class="pl-c1">0xF1</span>, <span class="pl-c1">0xB1</span>, <span class="pl-c1">0x8C</span>, <span class="pl-c1">0x5A</span>, <span class="pl-c1">0x9C</span>, <span class="pl-c1">0xAC</span>, <span class="pl-c1">0x2C</span>, <span class="pl-c1">0x28</span>, <span class="pl-c1">0x28</span>, <span class="pl-c1">0x28</span>, <span class="pl-c1">0x9C</span>, <span class="pl-c1">0xAC</span>, <span class="pl-c1">0x30</span>, <span class="pl-c1">0x18</span>, <span class="pl-c1">0xA8</span>,</td>
      </tr>
      <tr>
        <td id="L217" class="blob-num js-line-number" data-line-number="217"></td>
        <td id="LC217" class="blob-code blob-code-inner js-file-line">    <span class="pl-c1">0x98</span>, <span class="pl-c1">0x81</span>, <span class="pl-c1">0x28</span>, <span class="pl-c1">0x34</span>, <span class="pl-c1">0x3C</span>, <span class="pl-c1">0x97</span>, <span class="pl-c1">0x24</span>, <span class="pl-c1">0xA7</span>, <span class="pl-c1">0x28</span>, <span class="pl-c1">0x34</span>, <span class="pl-c1">0x3C</span>, <span class="pl-c1">0x9C</span>, <span class="pl-c1">0x24</span>, <span class="pl-c1">0xF2</span>, <span class="pl-c1">0xB0</span>, <span class="pl-c1">0x89</span>,</td>
      </tr>
      <tr>
        <td id="L218" class="blob-num js-line-number" data-line-number="218"></td>
        <td id="LC218" class="blob-code blob-code-inner js-file-line">    <span class="pl-c1">0xAC</span>, <span class="pl-c1">0x91</span>, <span class="pl-c1">0x2C</span>, <span class="pl-c1">0x4C</span>, <span class="pl-c1">0x6C</span>, <span class="pl-c1">0x8A</span>, <span class="pl-c1">0x9B</span>, <span class="pl-c1">0x2D</span>, <span class="pl-c1">0xD9</span>, <span class="pl-c1">0xD8</span>, <span class="pl-c1">0xD8</span>, <span class="pl-c1">0x51</span>, <span class="pl-c1">0xD9</span>, <span class="pl-c1">0xD8</span>, <span class="pl-c1">0xD8</span>, <span class="pl-c1">0x79</span>,</td>
      </tr>
      <tr>
        <td id="L219" class="blob-num js-line-number" data-line-number="219"></td>
        <td id="LC219" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L220" class="blob-num js-line-number" data-line-number="220"></td>
        <td id="LC220" class="blob-code blob-code-inner js-file-line">    <span class="pl-c"><span class="pl-c">//</span> bank 7, 138 bytes (remainder)</span></td>
      </tr>
      <tr>
        <td id="L221" class="blob-num js-line-number" data-line-number="221"></td>
        <td id="LC221" class="blob-code blob-code-inner js-file-line">    <span class="pl-c1">0xD9</span>, <span class="pl-c1">0xD8</span>, <span class="pl-c1">0xD8</span>, <span class="pl-c1">0xF1</span>, <span class="pl-c1">0x9E</span>, <span class="pl-c1">0x88</span>, <span class="pl-c1">0xA3</span>, <span class="pl-c1">0x31</span>, <span class="pl-c1">0xDA</span>, <span class="pl-c1">0xD8</span>, <span class="pl-c1">0xD8</span>, <span class="pl-c1">0x91</span>, <span class="pl-c1">0x2D</span>, <span class="pl-c1">0xD9</span>, <span class="pl-c1">0x28</span>, <span class="pl-c1">0xD8</span>,</td>
      </tr>
      <tr>
        <td id="L222" class="blob-num js-line-number" data-line-number="222"></td>
        <td id="LC222" class="blob-code blob-code-inner js-file-line">    <span class="pl-c1">0x4D</span>, <span class="pl-c1">0xD9</span>, <span class="pl-c1">0x48</span>, <span class="pl-c1">0xD8</span>, <span class="pl-c1">0x6D</span>, <span class="pl-c1">0xD9</span>, <span class="pl-c1">0x68</span>, <span class="pl-c1">0xD8</span>, <span class="pl-c1">0xB1</span>, <span class="pl-c1">0x83</span>, <span class="pl-c1">0x93</span>, <span class="pl-c1">0x35</span>, <span class="pl-c1">0x3D</span>, <span class="pl-c1">0x80</span>, <span class="pl-c1">0x25</span>, <span class="pl-c1">0xDA</span>,</td>
      </tr>
      <tr>
        <td id="L223" class="blob-num js-line-number" data-line-number="223"></td>
        <td id="LC223" class="blob-code blob-code-inner js-file-line">    <span class="pl-c1">0xD8</span>, <span class="pl-c1">0xD8</span>, <span class="pl-c1">0x85</span>, <span class="pl-c1">0x69</span>, <span class="pl-c1">0xDA</span>, <span class="pl-c1">0xD8</span>, <span class="pl-c1">0xD8</span>, <span class="pl-c1">0xB4</span>, <span class="pl-c1">0x93</span>, <span class="pl-c1">0x81</span>, <span class="pl-c1">0xA3</span>, <span class="pl-c1">0x28</span>, <span class="pl-c1">0x34</span>, <span class="pl-c1">0x3C</span>, <span class="pl-c1">0xF3</span>, <span class="pl-c1">0xAB</span>,</td>
      </tr>
      <tr>
        <td id="L224" class="blob-num js-line-number" data-line-number="224"></td>
        <td id="LC224" class="blob-code blob-code-inner js-file-line">    <span class="pl-c1">0x8B</span>, <span class="pl-c1">0xF8</span>, <span class="pl-c1">0xA3</span>, <span class="pl-c1">0x91</span>, <span class="pl-c1">0xB6</span>, <span class="pl-c1">0x09</span>, <span class="pl-c1">0xB4</span>, <span class="pl-c1">0xD9</span>, <span class="pl-c1">0xAB</span>, <span class="pl-c1">0xDE</span>, <span class="pl-c1">0xFA</span>, <span class="pl-c1">0xB0</span>, <span class="pl-c1">0x87</span>, <span class="pl-c1">0x9C</span>, <span class="pl-c1">0xB9</span>, <span class="pl-c1">0xA3</span>,</td>
      </tr>
      <tr>
        <td id="L225" class="blob-num js-line-number" data-line-number="225"></td>
        <td id="LC225" class="blob-code blob-code-inner js-file-line">    <span class="pl-c1">0xDD</span>, <span class="pl-c1">0xF1</span>, <span class="pl-c1">0xA3</span>, <span class="pl-c1">0xA3</span>, <span class="pl-c1">0xA3</span>, <span class="pl-c1">0xA3</span>, <span class="pl-c1">0x95</span>, <span class="pl-c1">0xF1</span>, <span class="pl-c1">0xA3</span>, <span class="pl-c1">0xA3</span>, <span class="pl-c1">0xA3</span>, <span class="pl-c1">0x9D</span>, <span class="pl-c1">0xF1</span>, <span class="pl-c1">0xA3</span>, <span class="pl-c1">0xA3</span>, <span class="pl-c1">0xA3</span>,</td>
      </tr>
      <tr>
        <td id="L226" class="blob-num js-line-number" data-line-number="226"></td>
        <td id="LC226" class="blob-code blob-code-inner js-file-line">    <span class="pl-c1">0xA3</span>, <span class="pl-c1">0xF2</span>, <span class="pl-c1">0xA3</span>, <span class="pl-c1">0xB4</span>, <span class="pl-c1">0x90</span>, <span class="pl-c1">0x80</span>, <span class="pl-c1">0xF2</span>, <span class="pl-c1">0xA3</span>, <span class="pl-c1">0xA3</span>, <span class="pl-c1">0xA3</span>, <span class="pl-c1">0xA3</span>, <span class="pl-c1">0xA3</span>, <span class="pl-c1">0xA3</span>, <span class="pl-c1">0xA3</span>, <span class="pl-c1">0xA3</span>, <span class="pl-c1">0xA3</span>,</td>
      </tr>
      <tr>
        <td id="L227" class="blob-num js-line-number" data-line-number="227"></td>
        <td id="LC227" class="blob-code blob-code-inner js-file-line">    <span class="pl-c1">0xA3</span>, <span class="pl-c1">0xB2</span>, <span class="pl-c1">0xA3</span>, <span class="pl-c1">0xA3</span>, <span class="pl-c1">0xA3</span>, <span class="pl-c1">0xA3</span>, <span class="pl-c1">0xA3</span>, <span class="pl-c1">0xA3</span>, <span class="pl-c1">0xB0</span>, <span class="pl-c1">0x87</span>, <span class="pl-c1">0xB5</span>, <span class="pl-c1">0x99</span>, <span class="pl-c1">0xF1</span>, <span class="pl-c1">0xA3</span>, <span class="pl-c1">0xA3</span>, <span class="pl-c1">0xA3</span>,</td>
      </tr>
      <tr>
        <td id="L228" class="blob-num js-line-number" data-line-number="228"></td>
        <td id="LC228" class="blob-code blob-code-inner js-file-line">    <span class="pl-c1">0x98</span>, <span class="pl-c1">0xF1</span>, <span class="pl-c1">0xA3</span>, <span class="pl-c1">0xA3</span>, <span class="pl-c1">0xA3</span>, <span class="pl-c1">0xA3</span>, <span class="pl-c1">0x97</span>, <span class="pl-c1">0xA3</span>, <span class="pl-c1">0xA3</span>, <span class="pl-c1">0xA3</span>, <span class="pl-c1">0xA3</span>, <span class="pl-c1">0xF3</span>, <span class="pl-c1">0x9B</span>, <span class="pl-c1">0xA3</span>, <span class="pl-c1">0xA3</span>, <span class="pl-c1">0xDC</span>,</td>
      </tr>
      <tr>
        <td id="L229" class="blob-num js-line-number" data-line-number="229"></td>
        <td id="LC229" class="blob-code blob-code-inner js-file-line">    <span class="pl-c1">0xB9</span>, <span class="pl-c1">0xA7</span>, <span class="pl-c1">0xF1</span>, <span class="pl-c1">0x26</span>, <span class="pl-c1">0x26</span>, <span class="pl-c1">0x26</span>, <span class="pl-c1">0xD8</span>, <span class="pl-c1">0xD8</span>, <span class="pl-c1">0xFF</span></td>
      </tr>
      <tr>
        <td id="L230" class="blob-num js-line-number" data-line-number="230"></td>
        <td id="LC230" class="blob-code blob-code-inner js-file-line">};</td>
      </tr>
      <tr>
        <td id="L231" class="blob-num js-line-number" data-line-number="231"></td>
        <td id="LC231" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L232" class="blob-num js-line-number" data-line-number="232"></td>
        <td id="LC232" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L233" class="blob-num js-line-number" data-line-number="233"></td>
        <td id="LC233" class="blob-code blob-code-inner js-file-line"><span class="pl-c"><span class="pl-c">//</span> DMP FIFO update rate: 0x09 drops the FIFO rate down to 20 Hz. 0x07 is 25 Hz,</span></td>
      </tr>
      <tr>
        <td id="L234" class="blob-num js-line-number" data-line-number="234"></td>
        <td id="LC234" class="blob-code blob-code-inner js-file-line"><span class="pl-c"><span class="pl-c">//</span> 0x01 is 100Hz. Going faster than 100Hz (0x00=200Hz) tends to result in very</span></td>
      </tr>
      <tr>
        <td id="L235" class="blob-num js-line-number" data-line-number="235"></td>
        <td id="LC235" class="blob-code blob-code-inner js-file-line"><span class="pl-c"><span class="pl-c">//</span> noisy data.  DMP output frequency is calculated easily using this equation:</span></td>
      </tr>
      <tr>
        <td id="L236" class="blob-num js-line-number" data-line-number="236"></td>
        <td id="LC236" class="blob-code blob-code-inner js-file-line"><span class="pl-c"><span class="pl-c">//</span> (200Hz / (1 + value))</span></td>
      </tr>
      <tr>
        <td id="L237" class="blob-num js-line-number" data-line-number="237"></td>
        <td id="LC237" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L238" class="blob-num js-line-number" data-line-number="238"></td>
        <td id="LC238" class="blob-code blob-code-inner js-file-line"><span class="pl-c"><span class="pl-c">//</span> It is important to make sure the host processor can keep up with reading and</span></td>
      </tr>
      <tr>
        <td id="L239" class="blob-num js-line-number" data-line-number="239"></td>
        <td id="LC239" class="blob-code blob-code-inner js-file-line"><span class="pl-c"><span class="pl-c">//</span> processing the FIFO output at the desired rate. Handling FIFO overflow</span></td>
      </tr>
      <tr>
        <td id="L240" class="blob-num js-line-number" data-line-number="240"></td>
        <td id="LC240" class="blob-code blob-code-inner js-file-line"><span class="pl-c"><span class="pl-c">//</span> cleanly is also a good idea.  thanks to Noah Zerkin for piecing this stuff</span></td>
      </tr>
      <tr>
        <td id="L241" class="blob-num js-line-number" data-line-number="241"></td>
        <td id="LC241" class="blob-code blob-code-inner js-file-line"><span class="pl-c"><span class="pl-c">//</span> together!</span></td>
      </tr>
      <tr>
        <td id="L242" class="blob-num js-line-number" data-line-number="242"></td>
        <td id="LC242" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L243" class="blob-num js-line-number" data-line-number="243"></td>
        <td id="LC243" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">ifndef</span> DMP_FIFO_RATE</td>
      </tr>
      <tr>
        <td id="L244" class="blob-num js-line-number" data-line-number="244"></td>
        <td id="LC244" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">DMP_FIFO_RATE</span>	<span class="pl-c1">1</span></td>
      </tr>
      <tr>
        <td id="L245" class="blob-num js-line-number" data-line-number="245"></td>
        <td id="LC245" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">endif</span></td>
      </tr>
      <tr>
        <td id="L246" class="blob-num js-line-number" data-line-number="246"></td>
        <td id="LC246" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L247" class="blob-num js-line-number" data-line-number="247"></td>
        <td id="LC247" class="blob-code blob-code-inner js-file-line"><span class="pl-k">const</span> prog_uchar dmpConfig[MPU6050_DMP_CONFIG_SIZE] PROGMEM = {</td>
      </tr>
      <tr>
        <td id="L248" class="blob-num js-line-number" data-line-number="248"></td>
        <td id="LC248" class="blob-code blob-code-inner js-file-line"><span class="pl-c"><span class="pl-c">//</span>  BANK    OFFSET  LENGTH  [DATA]</span></td>
      </tr>
      <tr>
        <td id="L249" class="blob-num js-line-number" data-line-number="249"></td>
        <td id="LC249" class="blob-code blob-code-inner js-file-line">    <span class="pl-c1">0x03</span>,   <span class="pl-c1">0x7B</span>,   <span class="pl-c1">0x03</span>,   <span class="pl-c1">0x4C</span>, <span class="pl-c1">0xCD</span>, <span class="pl-c1">0x6C</span>,         <span class="pl-c"><span class="pl-c">//</span> FCFG_1 inv_set_gyro_calibration</span></td>
      </tr>
      <tr>
        <td id="L250" class="blob-num js-line-number" data-line-number="250"></td>
        <td id="LC250" class="blob-code blob-code-inner js-file-line">    <span class="pl-c1">0x03</span>,   <span class="pl-c1">0xAB</span>,   <span class="pl-c1">0x03</span>,   <span class="pl-c1">0x36</span>, <span class="pl-c1">0x56</span>, <span class="pl-c1">0x76</span>,         <span class="pl-c"><span class="pl-c">//</span> FCFG_3 inv_set_gyro_calibration</span></td>
      </tr>
      <tr>
        <td id="L251" class="blob-num js-line-number" data-line-number="251"></td>
        <td id="LC251" class="blob-code blob-code-inner js-file-line">    <span class="pl-c1">0x00</span>,   <span class="pl-c1">0x68</span>,   <span class="pl-c1">0x04</span>,   <span class="pl-c1">0x02</span>, <span class="pl-c1">0xCB</span>, <span class="pl-c1">0x47</span>, <span class="pl-c1">0xA2</span>,   <span class="pl-c"><span class="pl-c">//</span> D_0_104 inv_set_gyro_calibration</span></td>
      </tr>
      <tr>
        <td id="L252" class="blob-num js-line-number" data-line-number="252"></td>
        <td id="LC252" class="blob-code blob-code-inner js-file-line">    <span class="pl-c1">0x02</span>,   <span class="pl-c1">0x18</span>,   <span class="pl-c1">0x04</span>,   <span class="pl-c1">0x00</span>, <span class="pl-c1">0x05</span>, <span class="pl-c1">0x8B</span>, <span class="pl-c1">0xC1</span>,   <span class="pl-c"><span class="pl-c">//</span> D_0_24 inv_set_gyro_calibration</span></td>
      </tr>
      <tr>
        <td id="L253" class="blob-num js-line-number" data-line-number="253"></td>
        <td id="LC253" class="blob-code blob-code-inner js-file-line">    <span class="pl-c1">0x01</span>,   <span class="pl-c1">0x0C</span>,   <span class="pl-c1">0x04</span>,   <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>,   <span class="pl-c"><span class="pl-c">//</span> D_1_152 inv_set_accel_calibration</span></td>
      </tr>
      <tr>
        <td id="L254" class="blob-num js-line-number" data-line-number="254"></td>
        <td id="LC254" class="blob-code blob-code-inner js-file-line">    <span class="pl-c1">0x03</span>,   <span class="pl-c1">0x7F</span>,   <span class="pl-c1">0x06</span>,   <span class="pl-c1">0x0C</span>, <span class="pl-c1">0xC9</span>, <span class="pl-c1">0x2C</span>, <span class="pl-c1">0x97</span>, <span class="pl-c1">0x97</span>, <span class="pl-c1">0x97</span>, <span class="pl-c"><span class="pl-c">//</span> FCFG_2 inv_set_accel_calibration</span></td>
      </tr>
      <tr>
        <td id="L255" class="blob-num js-line-number" data-line-number="255"></td>
        <td id="LC255" class="blob-code blob-code-inner js-file-line">    <span class="pl-c1">0x03</span>,   <span class="pl-c1">0x89</span>,   <span class="pl-c1">0x03</span>,   <span class="pl-c1">0x26</span>, <span class="pl-c1">0x46</span>, <span class="pl-c1">0x66</span>,         <span class="pl-c"><span class="pl-c">//</span> FCFG_7 inv_set_accel_calibration</span></td>
      </tr>
      <tr>
        <td id="L256" class="blob-num js-line-number" data-line-number="256"></td>
        <td id="LC256" class="blob-code blob-code-inner js-file-line">    <span class="pl-c1">0x00</span>,   <span class="pl-c1">0x6C</span>,   <span class="pl-c1">0x02</span>,   <span class="pl-c1">0x20</span>, <span class="pl-c1">0x00</span>,               <span class="pl-c"><span class="pl-c">//</span> D_0_108 inv_set_accel_calibration</span></td>
      </tr>
      <tr>
        <td id="L257" class="blob-num js-line-number" data-line-number="257"></td>
        <td id="LC257" class="blob-code blob-code-inner js-file-line">    <span class="pl-c1">0x02</span>,   <span class="pl-c1">0x40</span>,   <span class="pl-c1">0x04</span>,   <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>,   <span class="pl-c"><span class="pl-c">//</span> CPASS_MTX_00 inv_set_compass_calibration</span></td>
      </tr>
      <tr>
        <td id="L258" class="blob-num js-line-number" data-line-number="258"></td>
        <td id="LC258" class="blob-code blob-code-inner js-file-line">    <span class="pl-c1">0x02</span>,   <span class="pl-c1">0x44</span>,   <span class="pl-c1">0x04</span>,   <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>,   <span class="pl-c"><span class="pl-c">//</span> CPASS_MTX_01</span></td>
      </tr>
      <tr>
        <td id="L259" class="blob-num js-line-number" data-line-number="259"></td>
        <td id="LC259" class="blob-code blob-code-inner js-file-line">    <span class="pl-c1">0x02</span>,   <span class="pl-c1">0x48</span>,   <span class="pl-c1">0x04</span>,   <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>,   <span class="pl-c"><span class="pl-c">//</span> CPASS_MTX_02</span></td>
      </tr>
      <tr>
        <td id="L260" class="blob-num js-line-number" data-line-number="260"></td>
        <td id="LC260" class="blob-code blob-code-inner js-file-line">    <span class="pl-c1">0x02</span>,   <span class="pl-c1">0x4C</span>,   <span class="pl-c1">0x04</span>,   <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>,   <span class="pl-c"><span class="pl-c">//</span> CPASS_MTX_10</span></td>
      </tr>
      <tr>
        <td id="L261" class="blob-num js-line-number" data-line-number="261"></td>
        <td id="LC261" class="blob-code blob-code-inner js-file-line">    <span class="pl-c1">0x02</span>,   <span class="pl-c1">0x50</span>,   <span class="pl-c1">0x04</span>,   <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>,   <span class="pl-c"><span class="pl-c">//</span> CPASS_MTX_11</span></td>
      </tr>
      <tr>
        <td id="L262" class="blob-num js-line-number" data-line-number="262"></td>
        <td id="LC262" class="blob-code blob-code-inner js-file-line">    <span class="pl-c1">0x02</span>,   <span class="pl-c1">0x54</span>,   <span class="pl-c1">0x04</span>,   <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>,   <span class="pl-c"><span class="pl-c">//</span> CPASS_MTX_12</span></td>
      </tr>
      <tr>
        <td id="L263" class="blob-num js-line-number" data-line-number="263"></td>
        <td id="LC263" class="blob-code blob-code-inner js-file-line">    <span class="pl-c1">0x02</span>,   <span class="pl-c1">0x58</span>,   <span class="pl-c1">0x04</span>,   <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>,   <span class="pl-c"><span class="pl-c">//</span> CPASS_MTX_20</span></td>
      </tr>
      <tr>
        <td id="L264" class="blob-num js-line-number" data-line-number="264"></td>
        <td id="LC264" class="blob-code blob-code-inner js-file-line">    <span class="pl-c1">0x02</span>,   <span class="pl-c1">0x5C</span>,   <span class="pl-c1">0x04</span>,   <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>,   <span class="pl-c"><span class="pl-c">//</span> CPASS_MTX_21</span></td>
      </tr>
      <tr>
        <td id="L265" class="blob-num js-line-number" data-line-number="265"></td>
        <td id="LC265" class="blob-code blob-code-inner js-file-line">    <span class="pl-c1">0x02</span>,   <span class="pl-c1">0xBC</span>,   <span class="pl-c1">0x04</span>,   <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>,   <span class="pl-c"><span class="pl-c">//</span> CPASS_MTX_22</span></td>
      </tr>
      <tr>
        <td id="L266" class="blob-num js-line-number" data-line-number="266"></td>
        <td id="LC266" class="blob-code blob-code-inner js-file-line">    <span class="pl-c1">0x01</span>,   <span class="pl-c1">0xEC</span>,   <span class="pl-c1">0x04</span>,   <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x40</span>, <span class="pl-c1">0x00</span>,   <span class="pl-c"><span class="pl-c">//</span> D_1_236 inv_apply_endian_accel</span></td>
      </tr>
      <tr>
        <td id="L267" class="blob-num js-line-number" data-line-number="267"></td>
        <td id="LC267" class="blob-code blob-code-inner js-file-line">    <span class="pl-c1">0x03</span>,   <span class="pl-c1">0x7F</span>,   <span class="pl-c1">0x06</span>,   <span class="pl-c1">0x0C</span>, <span class="pl-c1">0xC9</span>, <span class="pl-c1">0x2C</span>, <span class="pl-c1">0x97</span>, <span class="pl-c1">0x97</span>, <span class="pl-c1">0x97</span>, <span class="pl-c"><span class="pl-c">//</span> FCFG_2 inv_set_mpu_sensors</span></td>
      </tr>
      <tr>
        <td id="L268" class="blob-num js-line-number" data-line-number="268"></td>
        <td id="LC268" class="blob-code blob-code-inner js-file-line">    <span class="pl-c1">0x04</span>,   <span class="pl-c1">0x02</span>,   <span class="pl-c1">0x03</span>,   <span class="pl-c1">0x0D</span>, <span class="pl-c1">0x35</span>, <span class="pl-c1">0x5D</span>,         <span class="pl-c"><span class="pl-c">//</span> CFG_MOTION_BIAS inv_turn_on_bias_from_no_motion</span></td>
      </tr>
      <tr>
        <td id="L269" class="blob-num js-line-number" data-line-number="269"></td>
        <td id="LC269" class="blob-code blob-code-inner js-file-line">    <span class="pl-c1">0x04</span>,   <span class="pl-c1">0x09</span>,   <span class="pl-c1">0x04</span>,   <span class="pl-c1">0x87</span>, <span class="pl-c1">0x2D</span>, <span class="pl-c1">0x35</span>, <span class="pl-c1">0x3D</span>,   <span class="pl-c"><span class="pl-c">//</span> FCFG_5 inv_set_bias_update</span></td>
      </tr>
      <tr>
        <td id="L270" class="blob-num js-line-number" data-line-number="270"></td>
        <td id="LC270" class="blob-code blob-code-inner js-file-line">    <span class="pl-c1">0x00</span>,   <span class="pl-c1">0xA3</span>,   <span class="pl-c1">0x01</span>,   <span class="pl-c1">0x00</span>,                     <span class="pl-c"><span class="pl-c">//</span> D_0_163 inv_set_dead_zone</span></td>
      </tr>
      <tr>
        <td id="L271" class="blob-num js-line-number" data-line-number="271"></td>
        <td id="LC271" class="blob-code blob-code-inner js-file-line">                 <span class="pl-c"><span class="pl-c">//</span> SPECIAL 0x01 = enable interrupts</span></td>
      </tr>
      <tr>
        <td id="L272" class="blob-num js-line-number" data-line-number="272"></td>
        <td id="LC272" class="blob-code blob-code-inner js-file-line">    <span class="pl-c1">0x00</span>,   <span class="pl-c1">0x00</span>,   <span class="pl-c1">0x00</span>,   <span class="pl-c1">0x01</span>, <span class="pl-c"><span class="pl-c">//</span> SET INT_ENABLE at i=22, SPECIAL INSTRUCTION</span></td>
      </tr>
      <tr>
        <td id="L273" class="blob-num js-line-number" data-line-number="273"></td>
        <td id="LC273" class="blob-code blob-code-inner js-file-line">    <span class="pl-c1">0x07</span>,   <span class="pl-c1">0x86</span>,   <span class="pl-c1">0x01</span>,   <span class="pl-c1">0xFE</span>,                     <span class="pl-c"><span class="pl-c">//</span> CFG_6 inv_set_fifo_interupt</span></td>
      </tr>
      <tr>
        <td id="L274" class="blob-num js-line-number" data-line-number="274"></td>
        <td id="LC274" class="blob-code blob-code-inner js-file-line">    <span class="pl-c1">0x07</span>,   <span class="pl-c1">0x41</span>,   <span class="pl-c1">0x05</span>,   <span class="pl-c1">0xF1</span>, <span class="pl-c1">0x20</span>, <span class="pl-c1">0x28</span>, <span class="pl-c1">0x30</span>, <span class="pl-c1">0x38</span>, <span class="pl-c"><span class="pl-c">//</span> CFG_8 inv_send_quaternion</span></td>
      </tr>
      <tr>
        <td id="L275" class="blob-num js-line-number" data-line-number="275"></td>
        <td id="LC275" class="blob-code blob-code-inner js-file-line">    <span class="pl-c1">0x07</span>,   <span class="pl-c1">0x7E</span>,   <span class="pl-c1">0x01</span>,   <span class="pl-c1">0x30</span>,                     <span class="pl-c"><span class="pl-c">//</span> CFG_16 inv_set_footer</span></td>
      </tr>
      <tr>
        <td id="L276" class="blob-num js-line-number" data-line-number="276"></td>
        <td id="LC276" class="blob-code blob-code-inner js-file-line">    <span class="pl-c1">0x07</span>,   <span class="pl-c1">0x46</span>,   <span class="pl-c1">0x01</span>,   <span class="pl-c1">0x9A</span>,                     <span class="pl-c"><span class="pl-c">//</span> CFG_GYRO_SOURCE inv_send_gyro</span></td>
      </tr>
      <tr>
        <td id="L277" class="blob-num js-line-number" data-line-number="277"></td>
        <td id="LC277" class="blob-code blob-code-inner js-file-line">    <span class="pl-c1">0x07</span>,   <span class="pl-c1">0x47</span>,   <span class="pl-c1">0x04</span>,   <span class="pl-c1">0xF1</span>, <span class="pl-c1">0x28</span>, <span class="pl-c1">0x30</span>, <span class="pl-c1">0x38</span>,   <span class="pl-c"><span class="pl-c">//</span> CFG_9 inv_send_gyro -&gt; inv_construct3_fifo</span></td>
      </tr>
      <tr>
        <td id="L278" class="blob-num js-line-number" data-line-number="278"></td>
        <td id="LC278" class="blob-code blob-code-inner js-file-line">    <span class="pl-c1">0x07</span>,   <span class="pl-c1">0x6C</span>,   <span class="pl-c1">0x04</span>,   <span class="pl-c1">0xF1</span>, <span class="pl-c1">0x28</span>, <span class="pl-c1">0x30</span>, <span class="pl-c1">0x38</span>,   <span class="pl-c"><span class="pl-c">//</span> CFG_12 inv_send_accel -&gt; inv_construct3_fifo</span></td>
      </tr>
      <tr>
        <td id="L279" class="blob-num js-line-number" data-line-number="279"></td>
        <td id="LC279" class="blob-code blob-code-inner js-file-line">    <span class="pl-c1">0x02</span>,   <span class="pl-c1">0x16</span>,   <span class="pl-c1">0x02</span>,   <span class="pl-c1">0x00</span>, DMP_FIFO_RATE       <span class="pl-c"><span class="pl-c">//</span> D_0_22 inv_set_fifo_rate</span></td>
      </tr>
      <tr>
        <td id="L280" class="blob-num js-line-number" data-line-number="280"></td>
        <td id="LC280" class="blob-code blob-code-inner js-file-line">};</td>
      </tr>
      <tr>
        <td id="L281" class="blob-num js-line-number" data-line-number="281"></td>
        <td id="LC281" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L282" class="blob-num js-line-number" data-line-number="282"></td>
        <td id="LC282" class="blob-code blob-code-inner js-file-line"><span class="pl-k">const</span> prog_uchar dmpUpdates[MPU6050_DMP_UPDATES_SIZE] PROGMEM = {</td>
      </tr>
      <tr>
        <td id="L283" class="blob-num js-line-number" data-line-number="283"></td>
        <td id="LC283" class="blob-code blob-code-inner js-file-line">    <span class="pl-c1">0x01</span>,   <span class="pl-c1">0xB2</span>,   <span class="pl-c1">0x02</span>,   <span class="pl-c1">0xFF</span>, <span class="pl-c1">0xFF</span>,</td>
      </tr>
      <tr>
        <td id="L284" class="blob-num js-line-number" data-line-number="284"></td>
        <td id="LC284" class="blob-code blob-code-inner js-file-line">    <span class="pl-c1">0x01</span>,   <span class="pl-c1">0x90</span>,   <span class="pl-c1">0x04</span>,   <span class="pl-c1">0x09</span>, <span class="pl-c1">0x23</span>, <span class="pl-c1">0xA1</span>, <span class="pl-c1">0x35</span>,</td>
      </tr>
      <tr>
        <td id="L285" class="blob-num js-line-number" data-line-number="285"></td>
        <td id="LC285" class="blob-code blob-code-inner js-file-line">    <span class="pl-c1">0x01</span>,   <span class="pl-c1">0x6A</span>,   <span class="pl-c1">0x02</span>,   <span class="pl-c1">0x06</span>, <span class="pl-c1">0x00</span>,</td>
      </tr>
      <tr>
        <td id="L286" class="blob-num js-line-number" data-line-number="286"></td>
        <td id="LC286" class="blob-code blob-code-inner js-file-line">    <span class="pl-c1">0x01</span>,   <span class="pl-c1">0x60</span>,   <span class="pl-c1">0x08</span>,   <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>,</td>
      </tr>
      <tr>
        <td id="L287" class="blob-num js-line-number" data-line-number="287"></td>
        <td id="LC287" class="blob-code blob-code-inner js-file-line">    <span class="pl-c1">0x00</span>,   <span class="pl-c1">0x60</span>,   <span class="pl-c1">0x04</span>,   <span class="pl-c1">0x40</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>,</td>
      </tr>
      <tr>
        <td id="L288" class="blob-num js-line-number" data-line-number="288"></td>
        <td id="LC288" class="blob-code blob-code-inner js-file-line">    <span class="pl-c1">0x01</span>,   <span class="pl-c1">0x62</span>,   <span class="pl-c1">0x02</span>,   <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span>,</td>
      </tr>
      <tr>
        <td id="L289" class="blob-num js-line-number" data-line-number="289"></td>
        <td id="LC289" class="blob-code blob-code-inner js-file-line">    <span class="pl-c1">0x00</span>,   <span class="pl-c1">0x60</span>,   <span class="pl-c1">0x04</span>,   <span class="pl-c1">0x00</span>, <span class="pl-c1">0x40</span>, <span class="pl-c1">0x00</span>, <span class="pl-c1">0x00</span></td>
      </tr>
      <tr>
        <td id="L290" class="blob-num js-line-number" data-line-number="290"></td>
        <td id="LC290" class="blob-code blob-code-inner js-file-line">};</td>
      </tr>
      <tr>
        <td id="L291" class="blob-num js-line-number" data-line-number="291"></td>
        <td id="LC291" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L292" class="blob-num js-line-number" data-line-number="292"></td>
        <td id="LC292" class="blob-code blob-code-inner js-file-line"><span class="pl-c1">uint8_t</span> <span class="pl-en">MPU6050::dmpInitialize</span>() {</td>
      </tr>
      <tr>
        <td id="L293" class="blob-num js-line-number" data-line-number="293"></td>
        <td id="LC293" class="blob-code blob-code-inner js-file-line">    <span class="pl-c"><span class="pl-c">//</span> reset device</span></td>
      </tr>
      <tr>
        <td id="L294" class="blob-num js-line-number" data-line-number="294"></td>
        <td id="LC294" class="blob-code blob-code-inner js-file-line">    <span class="pl-c1">DEBUG_PRINTLN</span>(<span class="pl-c1">F</span>(<span class="pl-s"><span class="pl-pds">&quot;</span><span class="pl-cce">\n\n</span>Resetting MPU6050...<span class="pl-pds">&quot;</span></span>));</td>
      </tr>
      <tr>
        <td id="L295" class="blob-num js-line-number" data-line-number="295"></td>
        <td id="LC295" class="blob-code blob-code-inner js-file-line">    <span class="pl-c1">reset</span>();</td>
      </tr>
      <tr>
        <td id="L296" class="blob-num js-line-number" data-line-number="296"></td>
        <td id="LC296" class="blob-code blob-code-inner js-file-line">    <span class="pl-c1">usleep</span>(<span class="pl-c1">30000</span>); <span class="pl-c"><span class="pl-c">//</span> wait after reset</span></td>
      </tr>
      <tr>
        <td id="L297" class="blob-num js-line-number" data-line-number="297"></td>
        <td id="LC297" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L298" class="blob-num js-line-number" data-line-number="298"></td>
        <td id="LC298" class="blob-code blob-code-inner js-file-line">    <span class="pl-c"><span class="pl-c">//</span> enable sleep mode and wake cycle</span></td>
      </tr>
      <tr>
        <td id="L299" class="blob-num js-line-number" data-line-number="299"></td>
        <td id="LC299" class="blob-code blob-code-inner js-file-line">    <span class="pl-c"><span class="pl-c">/*</span>Serial.println(F(&quot;Enabling sleep mode...&quot;));</span></td>
      </tr>
      <tr>
        <td id="L300" class="blob-num js-line-number" data-line-number="300"></td>
        <td id="LC300" class="blob-code blob-code-inner js-file-line"><span class="pl-c">    setSleepEnabled(true);</span></td>
      </tr>
      <tr>
        <td id="L301" class="blob-num js-line-number" data-line-number="301"></td>
        <td id="LC301" class="blob-code blob-code-inner js-file-line"><span class="pl-c">    Serial.println(F(&quot;Enabling wake cycle...&quot;));</span></td>
      </tr>
      <tr>
        <td id="L302" class="blob-num js-line-number" data-line-number="302"></td>
        <td id="LC302" class="blob-code blob-code-inner js-file-line"><span class="pl-c">    setWakeCycleEnabled(true);<span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L303" class="blob-num js-line-number" data-line-number="303"></td>
        <td id="LC303" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L304" class="blob-num js-line-number" data-line-number="304"></td>
        <td id="LC304" class="blob-code blob-code-inner js-file-line">    <span class="pl-c"><span class="pl-c">//</span> disable sleep mode</span></td>
      </tr>
      <tr>
        <td id="L305" class="blob-num js-line-number" data-line-number="305"></td>
        <td id="LC305" class="blob-code blob-code-inner js-file-line">    <span class="pl-c1">DEBUG_PRINTLN</span>(<span class="pl-c1">F</span>(<span class="pl-s"><span class="pl-pds">&quot;</span>Disabling sleep mode...<span class="pl-pds">&quot;</span></span>));</td>
      </tr>
      <tr>
        <td id="L306" class="blob-num js-line-number" data-line-number="306"></td>
        <td id="LC306" class="blob-code blob-code-inner js-file-line">    <span class="pl-c1">setSleepEnabled</span>(<span class="pl-c1">false</span>);</td>
      </tr>
      <tr>
        <td id="L307" class="blob-num js-line-number" data-line-number="307"></td>
        <td id="LC307" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L308" class="blob-num js-line-number" data-line-number="308"></td>
        <td id="LC308" class="blob-code blob-code-inner js-file-line">    <span class="pl-c"><span class="pl-c">//</span> get MPU hardware revision</span></td>
      </tr>
      <tr>
        <td id="L309" class="blob-num js-line-number" data-line-number="309"></td>
        <td id="LC309" class="blob-code blob-code-inner js-file-line">    <span class="pl-c1">DEBUG_PRINTLN</span>(<span class="pl-c1">F</span>(<span class="pl-s"><span class="pl-pds">&quot;</span>Selecting user bank 16...<span class="pl-pds">&quot;</span></span>));</td>
      </tr>
      <tr>
        <td id="L310" class="blob-num js-line-number" data-line-number="310"></td>
        <td id="LC310" class="blob-code blob-code-inner js-file-line">    <span class="pl-c1">setMemoryBank</span>(<span class="pl-c1">0x10</span>, <span class="pl-c1">true</span>, <span class="pl-c1">true</span>);</td>
      </tr>
      <tr>
        <td id="L311" class="blob-num js-line-number" data-line-number="311"></td>
        <td id="LC311" class="blob-code blob-code-inner js-file-line">    <span class="pl-c1">DEBUG_PRINTLN</span>(<span class="pl-c1">F</span>(<span class="pl-s"><span class="pl-pds">&quot;</span>Selecting memory byte 6...<span class="pl-pds">&quot;</span></span>));</td>
      </tr>
      <tr>
        <td id="L312" class="blob-num js-line-number" data-line-number="312"></td>
        <td id="LC312" class="blob-code blob-code-inner js-file-line">    <span class="pl-c1">setMemoryStartAddress</span>(<span class="pl-c1">0x06</span>);</td>
      </tr>
      <tr>
        <td id="L313" class="blob-num js-line-number" data-line-number="313"></td>
        <td id="LC313" class="blob-code blob-code-inner js-file-line">    <span class="pl-c1">DEBUG_PRINTLN</span>(<span class="pl-c1">F</span>(<span class="pl-s"><span class="pl-pds">&quot;</span>Checking hardware revision...<span class="pl-pds">&quot;</span></span>));</td>
      </tr>
      <tr>
        <td id="L314" class="blob-num js-line-number" data-line-number="314"></td>
        <td id="LC314" class="blob-code blob-code-inner js-file-line">    <span class="pl-c1">uint8_t</span> hwRevision <span class="pl-smi">__attribute__</span>((__unused__)) = <span class="pl-c1">readMemoryByte</span>();</td>
      </tr>
      <tr>
        <td id="L315" class="blob-num js-line-number" data-line-number="315"></td>
        <td id="LC315" class="blob-code blob-code-inner js-file-line">    <span class="pl-c1">DEBUG_PRINT</span>(<span class="pl-c1">F</span>(<span class="pl-s"><span class="pl-pds">&quot;</span>Revision @ user[16][6] = <span class="pl-pds">&quot;</span></span>));</td>
      </tr>
      <tr>
        <td id="L316" class="blob-num js-line-number" data-line-number="316"></td>
        <td id="LC316" class="blob-code blob-code-inner js-file-line">    <span class="pl-c1">DEBUG_PRINTLNF</span>(hwRevision, HEX);</td>
      </tr>
      <tr>
        <td id="L317" class="blob-num js-line-number" data-line-number="317"></td>
        <td id="LC317" class="blob-code blob-code-inner js-file-line">    <span class="pl-c1">DEBUG_PRINTLN</span>(<span class="pl-c1">F</span>(<span class="pl-s"><span class="pl-pds">&quot;</span>Resetting memory bank selection to 0...<span class="pl-pds">&quot;</span></span>));</td>
      </tr>
      <tr>
        <td id="L318" class="blob-num js-line-number" data-line-number="318"></td>
        <td id="LC318" class="blob-code blob-code-inner js-file-line">    <span class="pl-c1">setMemoryBank</span>(<span class="pl-c1">0</span>, <span class="pl-c1">false</span>, <span class="pl-c1">false</span>);</td>
      </tr>
      <tr>
        <td id="L319" class="blob-num js-line-number" data-line-number="319"></td>
        <td id="LC319" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L320" class="blob-num js-line-number" data-line-number="320"></td>
        <td id="LC320" class="blob-code blob-code-inner js-file-line">    <span class="pl-c"><span class="pl-c">//</span> check OTP bank valid</span></td>
      </tr>
      <tr>
        <td id="L321" class="blob-num js-line-number" data-line-number="321"></td>
        <td id="LC321" class="blob-code blob-code-inner js-file-line">    <span class="pl-c1">DEBUG_PRINTLN</span>(<span class="pl-c1">F</span>(<span class="pl-s"><span class="pl-pds">&quot;</span>Reading OTP bank valid flag...<span class="pl-pds">&quot;</span></span>));</td>
      </tr>
      <tr>
        <td id="L322" class="blob-num js-line-number" data-line-number="322"></td>
        <td id="LC322" class="blob-code blob-code-inner js-file-line">    <span class="pl-c1">uint8_t</span> otpValid <span class="pl-smi">__attribute__</span>((__unused__)) = <span class="pl-c1">getOTPBankValid</span>();</td>
      </tr>
      <tr>
        <td id="L323" class="blob-num js-line-number" data-line-number="323"></td>
        <td id="LC323" class="blob-code blob-code-inner js-file-line">    <span class="pl-c1">DEBUG_PRINT</span>(<span class="pl-c1">F</span>(<span class="pl-s"><span class="pl-pds">&quot;</span>OTP bank is <span class="pl-pds">&quot;</span></span>));</td>
      </tr>
      <tr>
        <td id="L324" class="blob-num js-line-number" data-line-number="324"></td>
        <td id="LC324" class="blob-code blob-code-inner js-file-line">    <span class="pl-c1">DEBUG_PRINTLN</span>(otpValid ? <span class="pl-c1">F</span>(<span class="pl-s"><span class="pl-pds">&quot;</span>valid!<span class="pl-pds">&quot;</span></span>) : <span class="pl-c1">F</span>(<span class="pl-s"><span class="pl-pds">&quot;</span>invalid!<span class="pl-pds">&quot;</span></span>));</td>
      </tr>
      <tr>
        <td id="L325" class="blob-num js-line-number" data-line-number="325"></td>
        <td id="LC325" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L326" class="blob-num js-line-number" data-line-number="326"></td>
        <td id="LC326" class="blob-code blob-code-inner js-file-line">    <span class="pl-c"><span class="pl-c">//</span> get X/Y/Z gyro offsets</span></td>
      </tr>
      <tr>
        <td id="L327" class="blob-num js-line-number" data-line-number="327"></td>
        <td id="LC327" class="blob-code blob-code-inner js-file-line">    <span class="pl-c1">DEBUG_PRINTLN</span>(<span class="pl-c1">F</span>(<span class="pl-s"><span class="pl-pds">&quot;</span>Reading gyro offset values...<span class="pl-pds">&quot;</span></span>));</td>
      </tr>
      <tr>
        <td id="L328" class="blob-num js-line-number" data-line-number="328"></td>
        <td id="LC328" class="blob-code blob-code-inner js-file-line">    <span class="pl-c1">int8_t</span> xgOffset = <span class="pl-c1">getXGyroOffset</span>();</td>
      </tr>
      <tr>
        <td id="L329" class="blob-num js-line-number" data-line-number="329"></td>
        <td id="LC329" class="blob-code blob-code-inner js-file-line">    <span class="pl-c1">int8_t</span> ygOffset = <span class="pl-c1">getYGyroOffset</span>();</td>
      </tr>
      <tr>
        <td id="L330" class="blob-num js-line-number" data-line-number="330"></td>
        <td id="LC330" class="blob-code blob-code-inner js-file-line">    <span class="pl-c1">int8_t</span> zgOffset = <span class="pl-c1">getZGyroOffset</span>();</td>
      </tr>
      <tr>
        <td id="L331" class="blob-num js-line-number" data-line-number="331"></td>
        <td id="LC331" class="blob-code blob-code-inner js-file-line">    <span class="pl-c1">DEBUG_PRINT</span>(<span class="pl-c1">F</span>(<span class="pl-s"><span class="pl-pds">&quot;</span>X gyro offset = <span class="pl-pds">&quot;</span></span>));</td>
      </tr>
      <tr>
        <td id="L332" class="blob-num js-line-number" data-line-number="332"></td>
        <td id="LC332" class="blob-code blob-code-inner js-file-line">    <span class="pl-c1">DEBUG_PRINTLN</span>(xgOffset);</td>
      </tr>
      <tr>
        <td id="L333" class="blob-num js-line-number" data-line-number="333"></td>
        <td id="LC333" class="blob-code blob-code-inner js-file-line">    <span class="pl-c1">DEBUG_PRINT</span>(<span class="pl-c1">F</span>(<span class="pl-s"><span class="pl-pds">&quot;</span>Y gyro offset = <span class="pl-pds">&quot;</span></span>));</td>
      </tr>
      <tr>
        <td id="L334" class="blob-num js-line-number" data-line-number="334"></td>
        <td id="LC334" class="blob-code blob-code-inner js-file-line">    <span class="pl-c1">DEBUG_PRINTLN</span>(ygOffset);</td>
      </tr>
      <tr>
        <td id="L335" class="blob-num js-line-number" data-line-number="335"></td>
        <td id="LC335" class="blob-code blob-code-inner js-file-line">    <span class="pl-c1">DEBUG_PRINT</span>(<span class="pl-c1">F</span>(<span class="pl-s"><span class="pl-pds">&quot;</span>Z gyro offset = <span class="pl-pds">&quot;</span></span>));</td>
      </tr>
      <tr>
        <td id="L336" class="blob-num js-line-number" data-line-number="336"></td>
        <td id="LC336" class="blob-code blob-code-inner js-file-line">    <span class="pl-c1">DEBUG_PRINTLN</span>(zgOffset);</td>
      </tr>
      <tr>
        <td id="L337" class="blob-num js-line-number" data-line-number="337"></td>
        <td id="LC337" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L338" class="blob-num js-line-number" data-line-number="338"></td>
        <td id="LC338" class="blob-code blob-code-inner js-file-line">    <span class="pl-c"><span class="pl-c">//</span> setup weird slave stuff (?)</span></td>
      </tr>
      <tr>
        <td id="L339" class="blob-num js-line-number" data-line-number="339"></td>
        <td id="LC339" class="blob-code blob-code-inner js-file-line">    <span class="pl-c1">DEBUG_PRINTLN</span>(<span class="pl-c1">F</span>(<span class="pl-s"><span class="pl-pds">&quot;</span>Setting slave 0 address to 0x7F...<span class="pl-pds">&quot;</span></span>));</td>
      </tr>
      <tr>
        <td id="L340" class="blob-num js-line-number" data-line-number="340"></td>
        <td id="LC340" class="blob-code blob-code-inner js-file-line">    <span class="pl-c1">setSlaveAddress</span>(<span class="pl-c1">0</span>, <span class="pl-c1">0x7F</span>);</td>
      </tr>
      <tr>
        <td id="L341" class="blob-num js-line-number" data-line-number="341"></td>
        <td id="LC341" class="blob-code blob-code-inner js-file-line">    <span class="pl-c1">DEBUG_PRINTLN</span>(<span class="pl-c1">F</span>(<span class="pl-s"><span class="pl-pds">&quot;</span>Disabling I2C Master mode...<span class="pl-pds">&quot;</span></span>));</td>
      </tr>
      <tr>
        <td id="L342" class="blob-num js-line-number" data-line-number="342"></td>
        <td id="LC342" class="blob-code blob-code-inner js-file-line">    <span class="pl-c1">setI2CMasterModeEnabled</span>(<span class="pl-c1">false</span>);</td>
      </tr>
      <tr>
        <td id="L343" class="blob-num js-line-number" data-line-number="343"></td>
        <td id="LC343" class="blob-code blob-code-inner js-file-line">    <span class="pl-c1">DEBUG_PRINTLN</span>(<span class="pl-c1">F</span>(<span class="pl-s"><span class="pl-pds">&quot;</span>Setting slave 0 address to 0x68 (self)...<span class="pl-pds">&quot;</span></span>));</td>
      </tr>
      <tr>
        <td id="L344" class="blob-num js-line-number" data-line-number="344"></td>
        <td id="LC344" class="blob-code blob-code-inner js-file-line">    <span class="pl-c1">setSlaveAddress</span>(<span class="pl-c1">0</span>, <span class="pl-c1">0x68</span>);</td>
      </tr>
      <tr>
        <td id="L345" class="blob-num js-line-number" data-line-number="345"></td>
        <td id="LC345" class="blob-code blob-code-inner js-file-line">    <span class="pl-c1">DEBUG_PRINTLN</span>(<span class="pl-c1">F</span>(<span class="pl-s"><span class="pl-pds">&quot;</span>Resetting I2C Master control...<span class="pl-pds">&quot;</span></span>));</td>
      </tr>
      <tr>
        <td id="L346" class="blob-num js-line-number" data-line-number="346"></td>
        <td id="LC346" class="blob-code blob-code-inner js-file-line">    <span class="pl-c1">resetI2CMaster</span>();</td>
      </tr>
      <tr>
        <td id="L347" class="blob-num js-line-number" data-line-number="347"></td>
        <td id="LC347" class="blob-code blob-code-inner js-file-line">    <span class="pl-c1">usleep</span>(<span class="pl-c1">20000</span>);</td>
      </tr>
      <tr>
        <td id="L348" class="blob-num js-line-number" data-line-number="348"></td>
        <td id="LC348" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L349" class="blob-num js-line-number" data-line-number="349"></td>
        <td id="LC349" class="blob-code blob-code-inner js-file-line">    <span class="pl-c"><span class="pl-c">//</span> load DMP code into memory banks</span></td>
      </tr>
      <tr>
        <td id="L350" class="blob-num js-line-number" data-line-number="350"></td>
        <td id="LC350" class="blob-code blob-code-inner js-file-line">    <span class="pl-c1">DEBUG_PRINT</span>(<span class="pl-c1">F</span>(<span class="pl-s"><span class="pl-pds">&quot;</span>Writing DMP code to MPU memory banks (<span class="pl-pds">&quot;</span></span>));</td>
      </tr>
      <tr>
        <td id="L351" class="blob-num js-line-number" data-line-number="351"></td>
        <td id="LC351" class="blob-code blob-code-inner js-file-line">    <span class="pl-c1">DEBUG_PRINT</span>(MPU6050_DMP_CODE_SIZE);</td>
      </tr>
      <tr>
        <td id="L352" class="blob-num js-line-number" data-line-number="352"></td>
        <td id="LC352" class="blob-code blob-code-inner js-file-line">    <span class="pl-c1">DEBUG_PRINTLN</span>(<span class="pl-c1">F</span>(<span class="pl-s"><span class="pl-pds">&quot;</span> bytes)<span class="pl-pds">&quot;</span></span>));</td>
      </tr>
      <tr>
        <td id="L353" class="blob-num js-line-number" data-line-number="353"></td>
        <td id="LC353" class="blob-code blob-code-inner js-file-line">    <span class="pl-k">if</span> (<span class="pl-c1">writeProgMemoryBlock</span>(dmpMemory, MPU6050_DMP_CODE_SIZE)) {</td>
      </tr>
      <tr>
        <td id="L354" class="blob-num js-line-number" data-line-number="354"></td>
        <td id="LC354" class="blob-code blob-code-inner js-file-line">        <span class="pl-c1">printf</span>(<span class="pl-s"><span class="pl-pds">&quot;</span>Success! DMP code written and verified.<span class="pl-cce">\n</span><span class="pl-pds">&quot;</span></span>);</td>
      </tr>
      <tr>
        <td id="L355" class="blob-num js-line-number" data-line-number="355"></td>
        <td id="LC355" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L356" class="blob-num js-line-number" data-line-number="356"></td>
        <td id="LC356" class="blob-code blob-code-inner js-file-line">        <span class="pl-c"><span class="pl-c">//</span> write DMP configuration</span></td>
      </tr>
      <tr>
        <td id="L357" class="blob-num js-line-number" data-line-number="357"></td>
        <td id="LC357" class="blob-code blob-code-inner js-file-line">        <span class="pl-c1">DEBUG_PRINT</span>(<span class="pl-c1">F</span>(<span class="pl-s"><span class="pl-pds">&quot;</span>Writing DMP configuration to MPU memory banks (<span class="pl-pds">&quot;</span></span>));</td>
      </tr>
      <tr>
        <td id="L358" class="blob-num js-line-number" data-line-number="358"></td>
        <td id="LC358" class="blob-code blob-code-inner js-file-line">        <span class="pl-c1">DEBUG_PRINT</span>(MPU6050_DMP_CONFIG_SIZE);</td>
      </tr>
      <tr>
        <td id="L359" class="blob-num js-line-number" data-line-number="359"></td>
        <td id="LC359" class="blob-code blob-code-inner js-file-line">        <span class="pl-c1">DEBUG_PRINTLN</span>(<span class="pl-c1">F</span>(<span class="pl-s"><span class="pl-pds">&quot;</span> bytes in config def)<span class="pl-pds">&quot;</span></span>));</td>
      </tr>
      <tr>
        <td id="L360" class="blob-num js-line-number" data-line-number="360"></td>
        <td id="LC360" class="blob-code blob-code-inner js-file-line">        <span class="pl-k">if</span> (<span class="pl-c1">writeProgDMPConfigurationSet</span>(dmpConfig, MPU6050_DMP_CONFIG_SIZE)) {</td>
      </tr>
      <tr>
        <td id="L361" class="blob-num js-line-number" data-line-number="361"></td>
        <td id="LC361" class="blob-code blob-code-inner js-file-line">            <span class="pl-c1">printf</span>(<span class="pl-s"><span class="pl-pds">&quot;</span>Success! DMP configuration written and verified.<span class="pl-cce">\n</span><span class="pl-pds">&quot;</span></span>);</td>
      </tr>
      <tr>
        <td id="L362" class="blob-num js-line-number" data-line-number="362"></td>
        <td id="LC362" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L363" class="blob-num js-line-number" data-line-number="363"></td>
        <td id="LC363" class="blob-code blob-code-inner js-file-line">            <span class="pl-c1">DEBUG_PRINTLN</span>(<span class="pl-c1">F</span>(<span class="pl-s"><span class="pl-pds">&quot;</span>Setting clock source to Z Gyro...<span class="pl-pds">&quot;</span></span>));</td>
      </tr>
      <tr>
        <td id="L364" class="blob-num js-line-number" data-line-number="364"></td>
        <td id="LC364" class="blob-code blob-code-inner js-file-line">            <span class="pl-c1">setClockSource</span>(MPU6050_CLOCK_PLL_ZGYRO);</td>
      </tr>
      <tr>
        <td id="L365" class="blob-num js-line-number" data-line-number="365"></td>
        <td id="LC365" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L366" class="blob-num js-line-number" data-line-number="366"></td>
        <td id="LC366" class="blob-code blob-code-inner js-file-line">            <span class="pl-c1">DEBUG_PRINTLN</span>(<span class="pl-c1">F</span>(<span class="pl-s"><span class="pl-pds">&quot;</span>Setting DMP and FIFO_OFLOW interrupts enabled...<span class="pl-pds">&quot;</span></span>));</td>
      </tr>
      <tr>
        <td id="L367" class="blob-num js-line-number" data-line-number="367"></td>
        <td id="LC367" class="blob-code blob-code-inner js-file-line">            <span class="pl-c1">setIntEnabled</span>(<span class="pl-c1">0x12</span>);</td>
      </tr>
      <tr>
        <td id="L368" class="blob-num js-line-number" data-line-number="368"></td>
        <td id="LC368" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L369" class="blob-num js-line-number" data-line-number="369"></td>
        <td id="LC369" class="blob-code blob-code-inner js-file-line">            <span class="pl-c1">DEBUG_PRINTLN</span>(<span class="pl-c1">F</span>(<span class="pl-s"><span class="pl-pds">&quot;</span>Setting sample rate to 200Hz...<span class="pl-pds">&quot;</span></span>));</td>
      </tr>
      <tr>
        <td id="L370" class="blob-num js-line-number" data-line-number="370"></td>
        <td id="LC370" class="blob-code blob-code-inner js-file-line">            <span class="pl-c1">setRate</span>(<span class="pl-c1">4</span>); <span class="pl-c"><span class="pl-c">//</span> 1khz / (1 + 4) = 200 Hz</span></td>
      </tr>
      <tr>
        <td id="L371" class="blob-num js-line-number" data-line-number="371"></td>
        <td id="LC371" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L372" class="blob-num js-line-number" data-line-number="372"></td>
        <td id="LC372" class="blob-code blob-code-inner js-file-line">            <span class="pl-c1">DEBUG_PRINTLN</span>(<span class="pl-c1">F</span>(<span class="pl-s"><span class="pl-pds">&quot;</span>Setting external frame sync to TEMP_OUT_L[0]...<span class="pl-pds">&quot;</span></span>));</td>
      </tr>
      <tr>
        <td id="L373" class="blob-num js-line-number" data-line-number="373"></td>
        <td id="LC373" class="blob-code blob-code-inner js-file-line">            <span class="pl-c1">setExternalFrameSync</span>(MPU6050_EXT_SYNC_TEMP_OUT_L);</td>
      </tr>
      <tr>
        <td id="L374" class="blob-num js-line-number" data-line-number="374"></td>
        <td id="LC374" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L375" class="blob-num js-line-number" data-line-number="375"></td>
        <td id="LC375" class="blob-code blob-code-inner js-file-line">            <span class="pl-c1">DEBUG_PRINTLN</span>(<span class="pl-c1">F</span>(<span class="pl-s"><span class="pl-pds">&quot;</span>Setting DLPF bandwidth to 42Hz...<span class="pl-pds">&quot;</span></span>));</td>
      </tr>
      <tr>
        <td id="L376" class="blob-num js-line-number" data-line-number="376"></td>
        <td id="LC376" class="blob-code blob-code-inner js-file-line">            <span class="pl-c1">setDLPFMode</span>(MPU6050_DLPF_BW_42);</td>
      </tr>
      <tr>
        <td id="L377" class="blob-num js-line-number" data-line-number="377"></td>
        <td id="LC377" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L378" class="blob-num js-line-number" data-line-number="378"></td>
        <td id="LC378" class="blob-code blob-code-inner js-file-line">            <span class="pl-c1">DEBUG_PRINTLN</span>(<span class="pl-c1">F</span>(<span class="pl-s"><span class="pl-pds">&quot;</span>Setting gyro sensitivity to +/- 2000 deg/sec...<span class="pl-pds">&quot;</span></span>));</td>
      </tr>
      <tr>
        <td id="L379" class="blob-num js-line-number" data-line-number="379"></td>
        <td id="LC379" class="blob-code blob-code-inner js-file-line">            <span class="pl-c1">setFullScaleGyroRange</span>(MPU6050_GYRO_FS_2000);</td>
      </tr>
      <tr>
        <td id="L380" class="blob-num js-line-number" data-line-number="380"></td>
        <td id="LC380" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L381" class="blob-num js-line-number" data-line-number="381"></td>
        <td id="LC381" class="blob-code blob-code-inner js-file-line">            <span class="pl-c1">DEBUG_PRINTLN</span>(<span class="pl-c1">F</span>(<span class="pl-s"><span class="pl-pds">&quot;</span>Setting DMP configuration bytes (function unknown)...<span class="pl-pds">&quot;</span></span>));</td>
      </tr>
      <tr>
        <td id="L382" class="blob-num js-line-number" data-line-number="382"></td>
        <td id="LC382" class="blob-code blob-code-inner js-file-line">            <span class="pl-c1">setDMPConfig1</span>(<span class="pl-c1">0x03</span>);</td>
      </tr>
      <tr>
        <td id="L383" class="blob-num js-line-number" data-line-number="383"></td>
        <td id="LC383" class="blob-code blob-code-inner js-file-line">            <span class="pl-c1">setDMPConfig2</span>(<span class="pl-c1">0x00</span>);</td>
      </tr>
      <tr>
        <td id="L384" class="blob-num js-line-number" data-line-number="384"></td>
        <td id="LC384" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L385" class="blob-num js-line-number" data-line-number="385"></td>
        <td id="LC385" class="blob-code blob-code-inner js-file-line">            <span class="pl-c1">DEBUG_PRINTLN</span>(<span class="pl-c1">F</span>(<span class="pl-s"><span class="pl-pds">&quot;</span>Clearing OTP Bank flag...<span class="pl-pds">&quot;</span></span>));</td>
      </tr>
      <tr>
        <td id="L386" class="blob-num js-line-number" data-line-number="386"></td>
        <td id="LC386" class="blob-code blob-code-inner js-file-line">            <span class="pl-c1">setOTPBankValid</span>(<span class="pl-c1">false</span>);</td>
      </tr>
      <tr>
        <td id="L387" class="blob-num js-line-number" data-line-number="387"></td>
        <td id="LC387" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L388" class="blob-num js-line-number" data-line-number="388"></td>
        <td id="LC388" class="blob-code blob-code-inner js-file-line">            <span class="pl-c1">DEBUG_PRINTLN</span>(<span class="pl-c1">F</span>(<span class="pl-s"><span class="pl-pds">&quot;</span>Setting X/Y/Z gyro offsets to previous values...<span class="pl-pds">&quot;</span></span>));</td>
      </tr>
      <tr>
        <td id="L389" class="blob-num js-line-number" data-line-number="389"></td>
        <td id="LC389" class="blob-code blob-code-inner js-file-line">            <span class="pl-c1">setXGyroOffset</span>(xgOffset);</td>
      </tr>
      <tr>
        <td id="L390" class="blob-num js-line-number" data-line-number="390"></td>
        <td id="LC390" class="blob-code blob-code-inner js-file-line">            <span class="pl-c1">setYGyroOffset</span>(ygOffset);</td>
      </tr>
      <tr>
        <td id="L391" class="blob-num js-line-number" data-line-number="391"></td>
        <td id="LC391" class="blob-code blob-code-inner js-file-line">            <span class="pl-c1">setZGyroOffset</span>(zgOffset);</td>
      </tr>
      <tr>
        <td id="L392" class="blob-num js-line-number" data-line-number="392"></td>
        <td id="LC392" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L393" class="blob-num js-line-number" data-line-number="393"></td>
        <td id="LC393" class="blob-code blob-code-inner js-file-line">            <span class="pl-c1">DEBUG_PRINTLN</span>(<span class="pl-c1">F</span>(<span class="pl-s"><span class="pl-pds">&quot;</span>Setting X/Y/Z gyro user offsets to zero...<span class="pl-pds">&quot;</span></span>));</td>
      </tr>
      <tr>
        <td id="L394" class="blob-num js-line-number" data-line-number="394"></td>
        <td id="LC394" class="blob-code blob-code-inner js-file-line">            <span class="pl-c1">setXGyroOffsetUser</span>(<span class="pl-c1">0</span>);</td>
      </tr>
      <tr>
        <td id="L395" class="blob-num js-line-number" data-line-number="395"></td>
        <td id="LC395" class="blob-code blob-code-inner js-file-line">            <span class="pl-c1">setYGyroOffsetUser</span>(<span class="pl-c1">0</span>);</td>
      </tr>
      <tr>
        <td id="L396" class="blob-num js-line-number" data-line-number="396"></td>
        <td id="LC396" class="blob-code blob-code-inner js-file-line">            <span class="pl-c1">setZGyroOffsetUser</span>(<span class="pl-c1">0</span>);</td>
      </tr>
      <tr>
        <td id="L397" class="blob-num js-line-number" data-line-number="397"></td>
        <td id="LC397" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L398" class="blob-num js-line-number" data-line-number="398"></td>
        <td id="LC398" class="blob-code blob-code-inner js-file-line">            <span class="pl-c1">DEBUG_PRINTLN</span>(<span class="pl-c1">F</span>(<span class="pl-s"><span class="pl-pds">&quot;</span>Writing final memory update 1/7 (function unknown)...<span class="pl-pds">&quot;</span></span>));</td>
      </tr>
      <tr>
        <td id="L399" class="blob-num js-line-number" data-line-number="399"></td>
        <td id="LC399" class="blob-code blob-code-inner js-file-line">            <span class="pl-c1">uint8_t</span> dmpUpdate[<span class="pl-c1">16</span>], j;</td>
      </tr>
      <tr>
        <td id="L400" class="blob-num js-line-number" data-line-number="400"></td>
        <td id="LC400" class="blob-code blob-code-inner js-file-line">            <span class="pl-c1">uint16_t</span> pos = <span class="pl-c1">0</span>;</td>
      </tr>
      <tr>
        <td id="L401" class="blob-num js-line-number" data-line-number="401"></td>
        <td id="LC401" class="blob-code blob-code-inner js-file-line">            <span class="pl-k">for</span> (j = <span class="pl-c1">0</span>; j &lt; <span class="pl-c1">4</span> || j &lt; dmpUpdate[<span class="pl-c1">2</span>] + <span class="pl-c1">3</span>; j++, pos++) dmpUpdate[j] = <span class="pl-c1">pgm_read_byte</span>(&amp;dmpUpdates[pos]);</td>
      </tr>
      <tr>
        <td id="L402" class="blob-num js-line-number" data-line-number="402"></td>
        <td id="LC402" class="blob-code blob-code-inner js-file-line">            <span class="pl-c1">writeMemoryBlock</span>(dmpUpdate + <span class="pl-c1">3</span>, dmpUpdate[<span class="pl-c1">2</span>], dmpUpdate[<span class="pl-c1">0</span>], dmpUpdate[<span class="pl-c1">1</span>]);</td>
      </tr>
      <tr>
        <td id="L403" class="blob-num js-line-number" data-line-number="403"></td>
        <td id="LC403" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L404" class="blob-num js-line-number" data-line-number="404"></td>
        <td id="LC404" class="blob-code blob-code-inner js-file-line">            <span class="pl-c1">DEBUG_PRINTLN</span>(<span class="pl-c1">F</span>(<span class="pl-s"><span class="pl-pds">&quot;</span>Writing final memory update 2/7 (function unknown)...<span class="pl-pds">&quot;</span></span>));</td>
      </tr>
      <tr>
        <td id="L405" class="blob-num js-line-number" data-line-number="405"></td>
        <td id="LC405" class="blob-code blob-code-inner js-file-line">            <span class="pl-k">for</span> (j = <span class="pl-c1">0</span>; j &lt; <span class="pl-c1">4</span> || j &lt; dmpUpdate[<span class="pl-c1">2</span>] + <span class="pl-c1">3</span>; j++, pos++) dmpUpdate[j] = <span class="pl-c1">pgm_read_byte</span>(&amp;dmpUpdates[pos]);</td>
      </tr>
      <tr>
        <td id="L406" class="blob-num js-line-number" data-line-number="406"></td>
        <td id="LC406" class="blob-code blob-code-inner js-file-line">            <span class="pl-c1">writeMemoryBlock</span>(dmpUpdate + <span class="pl-c1">3</span>, dmpUpdate[<span class="pl-c1">2</span>], dmpUpdate[<span class="pl-c1">0</span>], dmpUpdate[<span class="pl-c1">1</span>]);</td>
      </tr>
      <tr>
        <td id="L407" class="blob-num js-line-number" data-line-number="407"></td>
        <td id="LC407" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L408" class="blob-num js-line-number" data-line-number="408"></td>
        <td id="LC408" class="blob-code blob-code-inner js-file-line">            <span class="pl-c1">DEBUG_PRINTLN</span>(<span class="pl-c1">F</span>(<span class="pl-s"><span class="pl-pds">&quot;</span>Resetting FIFO...<span class="pl-pds">&quot;</span></span>));</td>
      </tr>
      <tr>
        <td id="L409" class="blob-num js-line-number" data-line-number="409"></td>
        <td id="LC409" class="blob-code blob-code-inner js-file-line">            <span class="pl-c1">resetFIFO</span>();</td>
      </tr>
      <tr>
        <td id="L410" class="blob-num js-line-number" data-line-number="410"></td>
        <td id="LC410" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L411" class="blob-num js-line-number" data-line-number="411"></td>
        <td id="LC411" class="blob-code blob-code-inner js-file-line">            <span class="pl-c1">DEBUG_PRINTLN</span>(<span class="pl-c1">F</span>(<span class="pl-s"><span class="pl-pds">&quot;</span>Reading FIFO count...<span class="pl-pds">&quot;</span></span>));</td>
      </tr>
      <tr>
        <td id="L412" class="blob-num js-line-number" data-line-number="412"></td>
        <td id="LC412" class="blob-code blob-code-inner js-file-line">            <span class="pl-c1">uint8_t</span> fifoCount = <span class="pl-c1">getFIFOCount</span>();</td>
      </tr>
      <tr>
        <td id="L413" class="blob-num js-line-number" data-line-number="413"></td>
        <td id="LC413" class="blob-code blob-code-inner js-file-line">            <span class="pl-c1">uint8_t</span> fifoBuffer[<span class="pl-c1">128</span>];</td>
      </tr>
      <tr>
        <td id="L414" class="blob-num js-line-number" data-line-number="414"></td>
        <td id="LC414" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L415" class="blob-num js-line-number" data-line-number="415"></td>
        <td id="LC415" class="blob-code blob-code-inner js-file-line">            <span class="pl-c1">printf</span>(<span class="pl-s"><span class="pl-pds">&quot;</span>Current FIFO count=<span class="pl-c1">%d</span><span class="pl-cce">\n</span><span class="pl-pds">&quot;</span></span>, fifoCount);</td>
      </tr>
      <tr>
        <td id="L416" class="blob-num js-line-number" data-line-number="416"></td>
        <td id="LC416" class="blob-code blob-code-inner js-file-line">            <span class="pl-c1">DEBUG_PRINTLN</span>(fifoCount);</td>
      </tr>
      <tr>
        <td id="L417" class="blob-num js-line-number" data-line-number="417"></td>
        <td id="LC417" class="blob-code blob-code-inner js-file-line">            <span class="pl-k">if</span> (fifoCount &gt; <span class="pl-c1">0</span>)</td>
      </tr>
      <tr>
        <td id="L418" class="blob-num js-line-number" data-line-number="418"></td>
        <td id="LC418" class="blob-code blob-code-inner js-file-line">		<span class="pl-c1">getFIFOBytes</span>(fifoBuffer, fifoCount);</td>
      </tr>
      <tr>
        <td id="L419" class="blob-num js-line-number" data-line-number="419"></td>
        <td id="LC419" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L420" class="blob-num js-line-number" data-line-number="420"></td>
        <td id="LC420" class="blob-code blob-code-inner js-file-line">            <span class="pl-c1">DEBUG_PRINTLN</span>(<span class="pl-c1">F</span>(<span class="pl-s"><span class="pl-pds">&quot;</span>Setting motion detection threshold to 2...<span class="pl-pds">&quot;</span></span>));</td>
      </tr>
      <tr>
        <td id="L421" class="blob-num js-line-number" data-line-number="421"></td>
        <td id="LC421" class="blob-code blob-code-inner js-file-line">            <span class="pl-c1">setMotionDetectionThreshold</span>(<span class="pl-c1">2</span>);</td>
      </tr>
      <tr>
        <td id="L422" class="blob-num js-line-number" data-line-number="422"></td>
        <td id="LC422" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L423" class="blob-num js-line-number" data-line-number="423"></td>
        <td id="LC423" class="blob-code blob-code-inner js-file-line">            <span class="pl-c1">DEBUG_PRINTLN</span>(<span class="pl-c1">F</span>(<span class="pl-s"><span class="pl-pds">&quot;</span>Setting zero-motion detection threshold to 156...<span class="pl-pds">&quot;</span></span>));</td>
      </tr>
      <tr>
        <td id="L424" class="blob-num js-line-number" data-line-number="424"></td>
        <td id="LC424" class="blob-code blob-code-inner js-file-line">            <span class="pl-c1">setZeroMotionDetectionThreshold</span>(<span class="pl-c1">156</span>);</td>
      </tr>
      <tr>
        <td id="L425" class="blob-num js-line-number" data-line-number="425"></td>
        <td id="LC425" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L426" class="blob-num js-line-number" data-line-number="426"></td>
        <td id="LC426" class="blob-code blob-code-inner js-file-line">            <span class="pl-c1">DEBUG_PRINTLN</span>(<span class="pl-c1">F</span>(<span class="pl-s"><span class="pl-pds">&quot;</span>Setting motion detection duration to 80...<span class="pl-pds">&quot;</span></span>));</td>
      </tr>
      <tr>
        <td id="L427" class="blob-num js-line-number" data-line-number="427"></td>
        <td id="LC427" class="blob-code blob-code-inner js-file-line">            <span class="pl-c1">setMotionDetectionDuration</span>(<span class="pl-c1">80</span>);</td>
      </tr>
      <tr>
        <td id="L428" class="blob-num js-line-number" data-line-number="428"></td>
        <td id="LC428" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L429" class="blob-num js-line-number" data-line-number="429"></td>
        <td id="LC429" class="blob-code blob-code-inner js-file-line">            <span class="pl-c1">DEBUG_PRINTLN</span>(<span class="pl-c1">F</span>(<span class="pl-s"><span class="pl-pds">&quot;</span>Setting zero-motion detection duration to 0...<span class="pl-pds">&quot;</span></span>));</td>
      </tr>
      <tr>
        <td id="L430" class="blob-num js-line-number" data-line-number="430"></td>
        <td id="LC430" class="blob-code blob-code-inner js-file-line">            <span class="pl-c1">setZeroMotionDetectionDuration</span>(<span class="pl-c1">0</span>);</td>
      </tr>
      <tr>
        <td id="L431" class="blob-num js-line-number" data-line-number="431"></td>
        <td id="LC431" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L432" class="blob-num js-line-number" data-line-number="432"></td>
        <td id="LC432" class="blob-code blob-code-inner js-file-line">            <span class="pl-c1">DEBUG_PRINTLN</span>(<span class="pl-c1">F</span>(<span class="pl-s"><span class="pl-pds">&quot;</span>Resetting FIFO...<span class="pl-pds">&quot;</span></span>));</td>
      </tr>
      <tr>
        <td id="L433" class="blob-num js-line-number" data-line-number="433"></td>
        <td id="LC433" class="blob-code blob-code-inner js-file-line">            <span class="pl-c1">resetFIFO</span>();</td>
      </tr>
      <tr>
        <td id="L434" class="blob-num js-line-number" data-line-number="434"></td>
        <td id="LC434" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L435" class="blob-num js-line-number" data-line-number="435"></td>
        <td id="LC435" class="blob-code blob-code-inner js-file-line">            <span class="pl-c1">DEBUG_PRINTLN</span>(<span class="pl-c1">F</span>(<span class="pl-s"><span class="pl-pds">&quot;</span>Enabling FIFO...<span class="pl-pds">&quot;</span></span>));</td>
      </tr>
      <tr>
        <td id="L436" class="blob-num js-line-number" data-line-number="436"></td>
        <td id="LC436" class="blob-code blob-code-inner js-file-line">            <span class="pl-c1">setFIFOEnabled</span>(<span class="pl-c1">true</span>);</td>
      </tr>
      <tr>
        <td id="L437" class="blob-num js-line-number" data-line-number="437"></td>
        <td id="LC437" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L438" class="blob-num js-line-number" data-line-number="438"></td>
        <td id="LC438" class="blob-code blob-code-inner js-file-line">            <span class="pl-c1">DEBUG_PRINTLN</span>(<span class="pl-c1">F</span>(<span class="pl-s"><span class="pl-pds">&quot;</span>Enabling DMP...<span class="pl-pds">&quot;</span></span>));</td>
      </tr>
      <tr>
        <td id="L439" class="blob-num js-line-number" data-line-number="439"></td>
        <td id="LC439" class="blob-code blob-code-inner js-file-line">            <span class="pl-c1">setDMPEnabled</span>(<span class="pl-c1">true</span>);</td>
      </tr>
      <tr>
        <td id="L440" class="blob-num js-line-number" data-line-number="440"></td>
        <td id="LC440" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L441" class="blob-num js-line-number" data-line-number="441"></td>
        <td id="LC441" class="blob-code blob-code-inner js-file-line">            <span class="pl-c1">DEBUG_PRINTLN</span>(<span class="pl-c1">F</span>(<span class="pl-s"><span class="pl-pds">&quot;</span>Resetting DMP...<span class="pl-pds">&quot;</span></span>));</td>
      </tr>
      <tr>
        <td id="L442" class="blob-num js-line-number" data-line-number="442"></td>
        <td id="LC442" class="blob-code blob-code-inner js-file-line">            <span class="pl-c1">resetDMP</span>();</td>
      </tr>
      <tr>
        <td id="L443" class="blob-num js-line-number" data-line-number="443"></td>
        <td id="LC443" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L444" class="blob-num js-line-number" data-line-number="444"></td>
        <td id="LC444" class="blob-code blob-code-inner js-file-line">            <span class="pl-c1">DEBUG_PRINTLN</span>(<span class="pl-c1">F</span>(<span class="pl-s"><span class="pl-pds">&quot;</span>Writing final memory update 3/7 (function unknown)...<span class="pl-pds">&quot;</span></span>));</td>
      </tr>
      <tr>
        <td id="L445" class="blob-num js-line-number" data-line-number="445"></td>
        <td id="LC445" class="blob-code blob-code-inner js-file-line">            <span class="pl-k">for</span> (j = <span class="pl-c1">0</span>; j &lt; <span class="pl-c1">4</span> || j &lt; dmpUpdate[<span class="pl-c1">2</span>] + <span class="pl-c1">3</span>; j++, pos++) dmpUpdate[j] = <span class="pl-c1">pgm_read_byte</span>(&amp;dmpUpdates[pos]);</td>
      </tr>
      <tr>
        <td id="L446" class="blob-num js-line-number" data-line-number="446"></td>
        <td id="LC446" class="blob-code blob-code-inner js-file-line">            <span class="pl-c1">writeMemoryBlock</span>(dmpUpdate + <span class="pl-c1">3</span>, dmpUpdate[<span class="pl-c1">2</span>], dmpUpdate[<span class="pl-c1">0</span>], dmpUpdate[<span class="pl-c1">1</span>]);</td>
      </tr>
      <tr>
        <td id="L447" class="blob-num js-line-number" data-line-number="447"></td>
        <td id="LC447" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L448" class="blob-num js-line-number" data-line-number="448"></td>
        <td id="LC448" class="blob-code blob-code-inner js-file-line">            <span class="pl-c1">DEBUG_PRINTLN</span>(<span class="pl-c1">F</span>(<span class="pl-s"><span class="pl-pds">&quot;</span>Writing final memory update 4/7 (function unknown)...<span class="pl-pds">&quot;</span></span>));</td>
      </tr>
      <tr>
        <td id="L449" class="blob-num js-line-number" data-line-number="449"></td>
        <td id="LC449" class="blob-code blob-code-inner js-file-line">            <span class="pl-k">for</span> (j = <span class="pl-c1">0</span>; j &lt; <span class="pl-c1">4</span> || j &lt; dmpUpdate[<span class="pl-c1">2</span>] + <span class="pl-c1">3</span>; j++, pos++) dmpUpdate[j] = <span class="pl-c1">pgm_read_byte</span>(&amp;dmpUpdates[pos]);</td>
      </tr>
      <tr>
        <td id="L450" class="blob-num js-line-number" data-line-number="450"></td>
        <td id="LC450" class="blob-code blob-code-inner js-file-line">            <span class="pl-c1">writeMemoryBlock</span>(dmpUpdate + <span class="pl-c1">3</span>, dmpUpdate[<span class="pl-c1">2</span>], dmpUpdate[<span class="pl-c1">0</span>], dmpUpdate[<span class="pl-c1">1</span>]);</td>
      </tr>
      <tr>
        <td id="L451" class="blob-num js-line-number" data-line-number="451"></td>
        <td id="LC451" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L452" class="blob-num js-line-number" data-line-number="452"></td>
        <td id="LC452" class="blob-code blob-code-inner js-file-line">            <span class="pl-c1">DEBUG_PRINTLN</span>(<span class="pl-c1">F</span>(<span class="pl-s"><span class="pl-pds">&quot;</span>Writing final memory update 5/7 (function unknown)...<span class="pl-pds">&quot;</span></span>));</td>
      </tr>
      <tr>
        <td id="L453" class="blob-num js-line-number" data-line-number="453"></td>
        <td id="LC453" class="blob-code blob-code-inner js-file-line">            <span class="pl-k">for</span> (j = <span class="pl-c1">0</span>; j &lt; <span class="pl-c1">4</span> || j &lt; dmpUpdate[<span class="pl-c1">2</span>] + <span class="pl-c1">3</span>; j++, pos++) dmpUpdate[j] = <span class="pl-c1">pgm_read_byte</span>(&amp;dmpUpdates[pos]);</td>
      </tr>
      <tr>
        <td id="L454" class="blob-num js-line-number" data-line-number="454"></td>
        <td id="LC454" class="blob-code blob-code-inner js-file-line">            <span class="pl-c1">writeMemoryBlock</span>(dmpUpdate + <span class="pl-c1">3</span>, dmpUpdate[<span class="pl-c1">2</span>], dmpUpdate[<span class="pl-c1">0</span>], dmpUpdate[<span class="pl-c1">1</span>]);</td>
      </tr>
      <tr>
        <td id="L455" class="blob-num js-line-number" data-line-number="455"></td>
        <td id="LC455" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L456" class="blob-num js-line-number" data-line-number="456"></td>
        <td id="LC456" class="blob-code blob-code-inner js-file-line">            <span class="pl-c1">printf</span>(<span class="pl-s"><span class="pl-pds">&quot;</span>Waiting for FIFO count &gt; 2...<span class="pl-cce">\n</span><span class="pl-pds">&quot;</span></span>);</td>
      </tr>
      <tr>
        <td id="L457" class="blob-num js-line-number" data-line-number="457"></td>
        <td id="LC457" class="blob-code blob-code-inner js-file-line">            <span class="pl-k">while</span> ((fifoCount = <span class="pl-c1">getFIFOCount</span>()) &lt; <span class="pl-c1">3</span>);</td>
      </tr>
      <tr>
        <td id="L458" class="blob-num js-line-number" data-line-number="458"></td>
        <td id="LC458" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L459" class="blob-num js-line-number" data-line-number="459"></td>
        <td id="LC459" class="blob-code blob-code-inner js-file-line">            <span class="pl-c1">printf</span>(<span class="pl-s"><span class="pl-pds">&quot;</span>Current FIFO count=<span class="pl-c1">%d</span><span class="pl-pds">&quot;</span></span>,fifoCount);</td>
      </tr>
      <tr>
        <td id="L460" class="blob-num js-line-number" data-line-number="460"></td>
        <td id="LC460" class="blob-code blob-code-inner js-file-line">            <span class="pl-c1">DEBUG_PRINTLN</span>(fifoCount);</td>
      </tr>
      <tr>
        <td id="L461" class="blob-num js-line-number" data-line-number="461"></td>
        <td id="LC461" class="blob-code blob-code-inner js-file-line">            <span class="pl-c1">DEBUG_PRINTLN</span>(<span class="pl-c1">F</span>(<span class="pl-s"><span class="pl-pds">&quot;</span>Reading FIFO data...<span class="pl-pds">&quot;</span></span>));</td>
      </tr>
      <tr>
        <td id="L462" class="blob-num js-line-number" data-line-number="462"></td>
        <td id="LC462" class="blob-code blob-code-inner js-file-line">            <span class="pl-c1">getFIFOBytes</span>(fifoBuffer, fifoCount);</td>
      </tr>
      <tr>
        <td id="L463" class="blob-num js-line-number" data-line-number="463"></td>
        <td id="LC463" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L464" class="blob-num js-line-number" data-line-number="464"></td>
        <td id="LC464" class="blob-code blob-code-inner js-file-line">            <span class="pl-c1">DEBUG_PRINTLN</span>(<span class="pl-c1">F</span>(<span class="pl-s"><span class="pl-pds">&quot;</span>Reading interrupt status...<span class="pl-pds">&quot;</span></span>));</td>
      </tr>
      <tr>
        <td id="L465" class="blob-num js-line-number" data-line-number="465"></td>
        <td id="LC465" class="blob-code blob-code-inner js-file-line">            <span class="pl-c1">uint8_t</span> mpuIntStatus <span class="pl-smi">__attribute__</span>((__unused__)) = <span class="pl-c1">getIntStatus</span>();</td>
      </tr>
      <tr>
        <td id="L466" class="blob-num js-line-number" data-line-number="466"></td>
        <td id="LC466" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L467" class="blob-num js-line-number" data-line-number="467"></td>
        <td id="LC467" class="blob-code blob-code-inner js-file-line">            <span class="pl-c1">DEBUG_PRINT</span>(<span class="pl-c1">F</span>(<span class="pl-s"><span class="pl-pds">&quot;</span>Current interrupt status=<span class="pl-pds">&quot;</span></span>));</td>
      </tr>
      <tr>
        <td id="L468" class="blob-num js-line-number" data-line-number="468"></td>
        <td id="LC468" class="blob-code blob-code-inner js-file-line">            <span class="pl-c1">DEBUG_PRINTLNF</span>(mpuIntStatus, HEX);</td>
      </tr>
      <tr>
        <td id="L469" class="blob-num js-line-number" data-line-number="469"></td>
        <td id="LC469" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L470" class="blob-num js-line-number" data-line-number="470"></td>
        <td id="LC470" class="blob-code blob-code-inner js-file-line">            <span class="pl-c1">DEBUG_PRINTLN</span>(<span class="pl-c1">F</span>(<span class="pl-s"><span class="pl-pds">&quot;</span>Reading final memory update 6/7 (function unknown)...<span class="pl-pds">&quot;</span></span>));</td>
      </tr>
      <tr>
        <td id="L471" class="blob-num js-line-number" data-line-number="471"></td>
        <td id="LC471" class="blob-code blob-code-inner js-file-line">            <span class="pl-k">for</span> (j = <span class="pl-c1">0</span>; j &lt; <span class="pl-c1">4</span> || j &lt; dmpUpdate[<span class="pl-c1">2</span>] + <span class="pl-c1">3</span>; j++, pos++) dmpUpdate[j] = <span class="pl-c1">pgm_read_byte</span>(&amp;dmpUpdates[pos]);</td>
      </tr>
      <tr>
        <td id="L472" class="blob-num js-line-number" data-line-number="472"></td>
        <td id="LC472" class="blob-code blob-code-inner js-file-line">            <span class="pl-c1">readMemoryBlock</span>(dmpUpdate + <span class="pl-c1">3</span>, dmpUpdate[<span class="pl-c1">2</span>], dmpUpdate[<span class="pl-c1">0</span>], dmpUpdate[<span class="pl-c1">1</span>]);</td>
      </tr>
      <tr>
        <td id="L473" class="blob-num js-line-number" data-line-number="473"></td>
        <td id="LC473" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L474" class="blob-num js-line-number" data-line-number="474"></td>
        <td id="LC474" class="blob-code blob-code-inner js-file-line">            <span class="pl-c1">DEBUG_PRINTLN</span>(<span class="pl-c1">F</span>(<span class="pl-s"><span class="pl-pds">&quot;</span>Waiting for FIFO count &gt; 2...<span class="pl-pds">&quot;</span></span>));</td>
      </tr>
      <tr>
        <td id="L475" class="blob-num js-line-number" data-line-number="475"></td>
        <td id="LC475" class="blob-code blob-code-inner js-file-line">            <span class="pl-k">while</span> ((fifoCount = <span class="pl-c1">getFIFOCount</span>()) &lt; <span class="pl-c1">3</span>);</td>
      </tr>
      <tr>
        <td id="L476" class="blob-num js-line-number" data-line-number="476"></td>
        <td id="LC476" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L477" class="blob-num js-line-number" data-line-number="477"></td>
        <td id="LC477" class="blob-code blob-code-inner js-file-line">            <span class="pl-c1">DEBUG_PRINT</span>(<span class="pl-c1">F</span>(<span class="pl-s"><span class="pl-pds">&quot;</span>Current FIFO count=<span class="pl-pds">&quot;</span></span>));</td>
      </tr>
      <tr>
        <td id="L478" class="blob-num js-line-number" data-line-number="478"></td>
        <td id="LC478" class="blob-code blob-code-inner js-file-line">            <span class="pl-c1">DEBUG_PRINTLN</span>(fifoCount);</td>
      </tr>
      <tr>
        <td id="L479" class="blob-num js-line-number" data-line-number="479"></td>
        <td id="LC479" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L480" class="blob-num js-line-number" data-line-number="480"></td>
        <td id="LC480" class="blob-code blob-code-inner js-file-line">            <span class="pl-c1">DEBUG_PRINTLN</span>(<span class="pl-c1">F</span>(<span class="pl-s"><span class="pl-pds">&quot;</span>Reading FIFO data...<span class="pl-pds">&quot;</span></span>));</td>
      </tr>
      <tr>
        <td id="L481" class="blob-num js-line-number" data-line-number="481"></td>
        <td id="LC481" class="blob-code blob-code-inner js-file-line">            <span class="pl-c1">getFIFOBytes</span>(fifoBuffer, fifoCount);</td>
      </tr>
      <tr>
        <td id="L482" class="blob-num js-line-number" data-line-number="482"></td>
        <td id="LC482" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L483" class="blob-num js-line-number" data-line-number="483"></td>
        <td id="LC483" class="blob-code blob-code-inner js-file-line">            <span class="pl-c1">DEBUG_PRINTLN</span>(<span class="pl-c1">F</span>(<span class="pl-s"><span class="pl-pds">&quot;</span>Reading interrupt status...<span class="pl-pds">&quot;</span></span>));</td>
      </tr>
      <tr>
        <td id="L484" class="blob-num js-line-number" data-line-number="484"></td>
        <td id="LC484" class="blob-code blob-code-inner js-file-line">            mpuIntStatus = <span class="pl-c1">getIntStatus</span>();</td>
      </tr>
      <tr>
        <td id="L485" class="blob-num js-line-number" data-line-number="485"></td>
        <td id="LC485" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L486" class="blob-num js-line-number" data-line-number="486"></td>
        <td id="LC486" class="blob-code blob-code-inner js-file-line">            <span class="pl-c1">DEBUG_PRINT</span>(<span class="pl-c1">F</span>(<span class="pl-s"><span class="pl-pds">&quot;</span>Current interrupt status=<span class="pl-pds">&quot;</span></span>));</td>
      </tr>
      <tr>
        <td id="L487" class="blob-num js-line-number" data-line-number="487"></td>
        <td id="LC487" class="blob-code blob-code-inner js-file-line">            <span class="pl-c1">DEBUG_PRINTLNF</span>(mpuIntStatus, HEX);</td>
      </tr>
      <tr>
        <td id="L488" class="blob-num js-line-number" data-line-number="488"></td>
        <td id="LC488" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L489" class="blob-num js-line-number" data-line-number="489"></td>
        <td id="LC489" class="blob-code blob-code-inner js-file-line">            <span class="pl-c1">DEBUG_PRINTLN</span>(<span class="pl-c1">F</span>(<span class="pl-s"><span class="pl-pds">&quot;</span>Writing final memory update 7/7 (function unknown)...<span class="pl-pds">&quot;</span></span>));</td>
      </tr>
      <tr>
        <td id="L490" class="blob-num js-line-number" data-line-number="490"></td>
        <td id="LC490" class="blob-code blob-code-inner js-file-line">            <span class="pl-k">for</span> (j = <span class="pl-c1">0</span>; j &lt; <span class="pl-c1">4</span> || j &lt; dmpUpdate[<span class="pl-c1">2</span>] + <span class="pl-c1">3</span>; j++, pos++) dmpUpdate[j] = <span class="pl-c1">pgm_read_byte</span>(&amp;dmpUpdates[pos]);</td>
      </tr>
      <tr>
        <td id="L491" class="blob-num js-line-number" data-line-number="491"></td>
        <td id="LC491" class="blob-code blob-code-inner js-file-line">            <span class="pl-c1">writeMemoryBlock</span>(dmpUpdate + <span class="pl-c1">3</span>, dmpUpdate[<span class="pl-c1">2</span>], dmpUpdate[<span class="pl-c1">0</span>], dmpUpdate[<span class="pl-c1">1</span>]);</td>
      </tr>
      <tr>
        <td id="L492" class="blob-num js-line-number" data-line-number="492"></td>
        <td id="LC492" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L493" class="blob-num js-line-number" data-line-number="493"></td>
        <td id="LC493" class="blob-code blob-code-inner js-file-line">            <span class="pl-c1">DEBUG_PRINTLN</span>(<span class="pl-c1">F</span>(<span class="pl-s"><span class="pl-pds">&quot;</span>DMP is good to go! Finally.<span class="pl-pds">&quot;</span></span>));</td>
      </tr>
      <tr>
        <td id="L494" class="blob-num js-line-number" data-line-number="494"></td>
        <td id="LC494" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L495" class="blob-num js-line-number" data-line-number="495"></td>
        <td id="LC495" class="blob-code blob-code-inner js-file-line">            <span class="pl-c1">DEBUG_PRINTLN</span>(<span class="pl-c1">F</span>(<span class="pl-s"><span class="pl-pds">&quot;</span>Disabling DMP (you turn it on later)...<span class="pl-pds">&quot;</span></span>));</td>
      </tr>
      <tr>
        <td id="L496" class="blob-num js-line-number" data-line-number="496"></td>
        <td id="LC496" class="blob-code blob-code-inner js-file-line">            <span class="pl-c1">setDMPEnabled</span>(<span class="pl-c1">false</span>);</td>
      </tr>
      <tr>
        <td id="L497" class="blob-num js-line-number" data-line-number="497"></td>
        <td id="LC497" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L498" class="blob-num js-line-number" data-line-number="498"></td>
        <td id="LC498" class="blob-code blob-code-inner js-file-line">            <span class="pl-c1">DEBUG_PRINTLN</span>(<span class="pl-c1">F</span>(<span class="pl-s"><span class="pl-pds">&quot;</span>Setting up internal 42-byte (default) DMP packet buffer...<span class="pl-pds">&quot;</span></span>));</td>
      </tr>
      <tr>
        <td id="L499" class="blob-num js-line-number" data-line-number="499"></td>
        <td id="LC499" class="blob-code blob-code-inner js-file-line">            dmpPacketSize = <span class="pl-c1">42</span>;</td>
      </tr>
      <tr>
        <td id="L500" class="blob-num js-line-number" data-line-number="500"></td>
        <td id="LC500" class="blob-code blob-code-inner js-file-line">            <span class="pl-c"><span class="pl-c">/*</span>if ((dmpPacketBuffer = (uint8_t *)malloc(42)) == 0) {</span></td>
      </tr>
      <tr>
        <td id="L501" class="blob-num js-line-number" data-line-number="501"></td>
        <td id="LC501" class="blob-code blob-code-inner js-file-line"><span class="pl-c">                return 3; // TODO: proper error code for no memory</span></td>
      </tr>
      <tr>
        <td id="L502" class="blob-num js-line-number" data-line-number="502"></td>
        <td id="LC502" class="blob-code blob-code-inner js-file-line"><span class="pl-c">            }<span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L503" class="blob-num js-line-number" data-line-number="503"></td>
        <td id="LC503" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L504" class="blob-num js-line-number" data-line-number="504"></td>
        <td id="LC504" class="blob-code blob-code-inner js-file-line">            <span class="pl-c1">DEBUG_PRINTLN</span>(<span class="pl-c1">F</span>(<span class="pl-s"><span class="pl-pds">&quot;</span>Resetting FIFO and clearing INT status one last time...<span class="pl-pds">&quot;</span></span>));</td>
      </tr>
      <tr>
        <td id="L505" class="blob-num js-line-number" data-line-number="505"></td>
        <td id="LC505" class="blob-code blob-code-inner js-file-line">            <span class="pl-c1">resetFIFO</span>();</td>
      </tr>
      <tr>
        <td id="L506" class="blob-num js-line-number" data-line-number="506"></td>
        <td id="LC506" class="blob-code blob-code-inner js-file-line">            <span class="pl-c1">getIntStatus</span>();</td>
      </tr>
      <tr>
        <td id="L507" class="blob-num js-line-number" data-line-number="507"></td>
        <td id="LC507" class="blob-code blob-code-inner js-file-line">        } <span class="pl-k">else</span> {</td>
      </tr>
      <tr>
        <td id="L508" class="blob-num js-line-number" data-line-number="508"></td>
        <td id="LC508" class="blob-code blob-code-inner js-file-line">            <span class="pl-c1">DEBUG_PRINTLN</span>(<span class="pl-c1">F</span>(<span class="pl-s"><span class="pl-pds">&quot;</span>ERROR! DMP configuration verification failed.<span class="pl-pds">&quot;</span></span>));</td>
      </tr>
      <tr>
        <td id="L509" class="blob-num js-line-number" data-line-number="509"></td>
        <td id="LC509" class="blob-code blob-code-inner js-file-line">            <span class="pl-k">return</span> <span class="pl-c1">2</span>; <span class="pl-c"><span class="pl-c">//</span> configuration block loading failed</span></td>
      </tr>
      <tr>
        <td id="L510" class="blob-num js-line-number" data-line-number="510"></td>
        <td id="LC510" class="blob-code blob-code-inner js-file-line">        }</td>
      </tr>
      <tr>
        <td id="L511" class="blob-num js-line-number" data-line-number="511"></td>
        <td id="LC511" class="blob-code blob-code-inner js-file-line">    } <span class="pl-k">else</span> {</td>
      </tr>
      <tr>
        <td id="L512" class="blob-num js-line-number" data-line-number="512"></td>
        <td id="LC512" class="blob-code blob-code-inner js-file-line">        <span class="pl-c1">DEBUG_PRINTLN</span>(<span class="pl-c1">F</span>(<span class="pl-s"><span class="pl-pds">&quot;</span>ERROR! DMP code verification failed.<span class="pl-pds">&quot;</span></span>));</td>
      </tr>
      <tr>
        <td id="L513" class="blob-num js-line-number" data-line-number="513"></td>
        <td id="LC513" class="blob-code blob-code-inner js-file-line">        <span class="pl-k">return</span> <span class="pl-c1">1</span>; <span class="pl-c"><span class="pl-c">//</span> main binary block loading failed</span></td>
      </tr>
      <tr>
        <td id="L514" class="blob-num js-line-number" data-line-number="514"></td>
        <td id="LC514" class="blob-code blob-code-inner js-file-line">    }</td>
      </tr>
      <tr>
        <td id="L515" class="blob-num js-line-number" data-line-number="515"></td>
        <td id="LC515" class="blob-code blob-code-inner js-file-line">    <span class="pl-k">return</span> <span class="pl-c1">0</span>; <span class="pl-c"><span class="pl-c">//</span> success</span></td>
      </tr>
      <tr>
        <td id="L516" class="blob-num js-line-number" data-line-number="516"></td>
        <td id="LC516" class="blob-code blob-code-inner js-file-line">}</td>
      </tr>
      <tr>
        <td id="L517" class="blob-num js-line-number" data-line-number="517"></td>
        <td id="LC517" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L518" class="blob-num js-line-number" data-line-number="518"></td>
        <td id="LC518" class="blob-code blob-code-inner js-file-line"><span class="pl-k">bool</span> <span class="pl-en">MPU6050::dmpPacketAvailable</span>() {</td>
      </tr>
      <tr>
        <td id="L519" class="blob-num js-line-number" data-line-number="519"></td>
        <td id="LC519" class="blob-code blob-code-inner js-file-line">    <span class="pl-k">return</span> <span class="pl-c1">getFIFOCount</span>() &gt;= <span class="pl-c1">dmpGetFIFOPacketSize</span>();</td>
      </tr>
      <tr>
        <td id="L520" class="blob-num js-line-number" data-line-number="520"></td>
        <td id="LC520" class="blob-code blob-code-inner js-file-line">}</td>
      </tr>
      <tr>
        <td id="L521" class="blob-num js-line-number" data-line-number="521"></td>
        <td id="LC521" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L522" class="blob-num js-line-number" data-line-number="522"></td>
        <td id="LC522" class="blob-code blob-code-inner js-file-line"><span class="pl-c"><span class="pl-c">//</span> uint8_t MPU6050::dmpSetFIFORate(uint8_t fifoRate);</span></td>
      </tr>
      <tr>
        <td id="L523" class="blob-num js-line-number" data-line-number="523"></td>
        <td id="LC523" class="blob-code blob-code-inner js-file-line"><span class="pl-c"><span class="pl-c">//</span> uint8_t MPU6050::dmpGetFIFORate();</span></td>
      </tr>
      <tr>
        <td id="L524" class="blob-num js-line-number" data-line-number="524"></td>
        <td id="LC524" class="blob-code blob-code-inner js-file-line"><span class="pl-c"><span class="pl-c">//</span> uint8_t MPU6050::dmpGetSampleStepSizeMS();</span></td>
      </tr>
      <tr>
        <td id="L525" class="blob-num js-line-number" data-line-number="525"></td>
        <td id="LC525" class="blob-code blob-code-inner js-file-line"><span class="pl-c"><span class="pl-c">//</span> uint8_t MPU6050::dmpGetSampleFrequency();</span></td>
      </tr>
      <tr>
        <td id="L526" class="blob-num js-line-number" data-line-number="526"></td>
        <td id="LC526" class="blob-code blob-code-inner js-file-line"><span class="pl-c"><span class="pl-c">//</span> int32_t MPU6050::dmpDecodeTemperature(int8_t tempReg);</span></td>
      </tr>
      <tr>
        <td id="L527" class="blob-num js-line-number" data-line-number="527"></td>
        <td id="LC527" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L528" class="blob-num js-line-number" data-line-number="528"></td>
        <td id="LC528" class="blob-code blob-code-inner js-file-line"><span class="pl-c"><span class="pl-c">//</span>uint8_t MPU6050::dmpRegisterFIFORateProcess(inv_obj_func func, int16_t priority);</span></td>
      </tr>
      <tr>
        <td id="L529" class="blob-num js-line-number" data-line-number="529"></td>
        <td id="LC529" class="blob-code blob-code-inner js-file-line"><span class="pl-c"><span class="pl-c">//</span>uint8_t MPU6050::dmpUnregisterFIFORateProcess(inv_obj_func func);</span></td>
      </tr>
      <tr>
        <td id="L530" class="blob-num js-line-number" data-line-number="530"></td>
        <td id="LC530" class="blob-code blob-code-inner js-file-line"><span class="pl-c"><span class="pl-c">//</span>uint8_t MPU6050::dmpRunFIFORateProcesses();</span></td>
      </tr>
      <tr>
        <td id="L531" class="blob-num js-line-number" data-line-number="531"></td>
        <td id="LC531" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L532" class="blob-num js-line-number" data-line-number="532"></td>
        <td id="LC532" class="blob-code blob-code-inner js-file-line"><span class="pl-c"><span class="pl-c">//</span> uint8_t MPU6050::dmpSendQuaternion(uint_fast16_t accuracy);</span></td>
      </tr>
      <tr>
        <td id="L533" class="blob-num js-line-number" data-line-number="533"></td>
        <td id="LC533" class="blob-code blob-code-inner js-file-line"><span class="pl-c"><span class="pl-c">//</span> uint8_t MPU6050::dmpSendGyro(uint_fast16_t elements, uint_fast16_t accuracy);</span></td>
      </tr>
      <tr>
        <td id="L534" class="blob-num js-line-number" data-line-number="534"></td>
        <td id="LC534" class="blob-code blob-code-inner js-file-line"><span class="pl-c"><span class="pl-c">//</span> uint8_t MPU6050::dmpSendAccel(uint_fast16_t elements, uint_fast16_t accuracy);</span></td>
      </tr>
      <tr>
        <td id="L535" class="blob-num js-line-number" data-line-number="535"></td>
        <td id="LC535" class="blob-code blob-code-inner js-file-line"><span class="pl-c"><span class="pl-c">//</span> uint8_t MPU6050::dmpSendLinearAccel(uint_fast16_t elements, uint_fast16_t accuracy);</span></td>
      </tr>
      <tr>
        <td id="L536" class="blob-num js-line-number" data-line-number="536"></td>
        <td id="LC536" class="blob-code blob-code-inner js-file-line"><span class="pl-c"><span class="pl-c">//</span> uint8_t MPU6050::dmpSendLinearAccelInWorld(uint_fast16_t elements, uint_fast16_t accuracy);</span></td>
      </tr>
      <tr>
        <td id="L537" class="blob-num js-line-number" data-line-number="537"></td>
        <td id="LC537" class="blob-code blob-code-inner js-file-line"><span class="pl-c"><span class="pl-c">//</span> uint8_t MPU6050::dmpSendControlData(uint_fast16_t elements, uint_fast16_t accuracy);</span></td>
      </tr>
      <tr>
        <td id="L538" class="blob-num js-line-number" data-line-number="538"></td>
        <td id="LC538" class="blob-code blob-code-inner js-file-line"><span class="pl-c"><span class="pl-c">//</span> uint8_t MPU6050::dmpSendSensorData(uint_fast16_t elements, uint_fast16_t accuracy);</span></td>
      </tr>
      <tr>
        <td id="L539" class="blob-num js-line-number" data-line-number="539"></td>
        <td id="LC539" class="blob-code blob-code-inner js-file-line"><span class="pl-c"><span class="pl-c">//</span> uint8_t MPU6050::dmpSendExternalSensorData(uint_fast16_t elements, uint_fast16_t accuracy);</span></td>
      </tr>
      <tr>
        <td id="L540" class="blob-num js-line-number" data-line-number="540"></td>
        <td id="LC540" class="blob-code blob-code-inner js-file-line"><span class="pl-c"><span class="pl-c">//</span> uint8_t MPU6050::dmpSendGravity(uint_fast16_t elements, uint_fast16_t accuracy);</span></td>
      </tr>
      <tr>
        <td id="L541" class="blob-num js-line-number" data-line-number="541"></td>
        <td id="LC541" class="blob-code blob-code-inner js-file-line"><span class="pl-c"><span class="pl-c">//</span> uint8_t MPU6050::dmpSendPacketNumber(uint_fast16_t accuracy);</span></td>
      </tr>
      <tr>
        <td id="L542" class="blob-num js-line-number" data-line-number="542"></td>
        <td id="LC542" class="blob-code blob-code-inner js-file-line"><span class="pl-c"><span class="pl-c">//</span> uint8_t MPU6050::dmpSendQuantizedAccel(uint_fast16_t elements, uint_fast16_t accuracy);</span></td>
      </tr>
      <tr>
        <td id="L543" class="blob-num js-line-number" data-line-number="543"></td>
        <td id="LC543" class="blob-code blob-code-inner js-file-line"><span class="pl-c"><span class="pl-c">//</span> uint8_t MPU6050::dmpSendEIS(uint_fast16_t elements, uint_fast16_t accuracy);</span></td>
      </tr>
      <tr>
        <td id="L544" class="blob-num js-line-number" data-line-number="544"></td>
        <td id="LC544" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L545" class="blob-num js-line-number" data-line-number="545"></td>
        <td id="LC545" class="blob-code blob-code-inner js-file-line"><span class="pl-c1">uint8_t</span> <span class="pl-en">MPU6050::dmpGetAccel</span>(<span class="pl-c1">int32_t</span> *data, <span class="pl-k">const</span> <span class="pl-c1">uint8_t</span>* packet) {</td>
      </tr>
      <tr>
        <td id="L546" class="blob-num js-line-number" data-line-number="546"></td>
        <td id="LC546" class="blob-code blob-code-inner js-file-line">    <span class="pl-c"><span class="pl-c">//</span> TODO: accommodate different arrangements of sent data (ONLY default supported now)</span></td>
      </tr>
      <tr>
        <td id="L547" class="blob-num js-line-number" data-line-number="547"></td>
        <td id="LC547" class="blob-code blob-code-inner js-file-line">    <span class="pl-k">if</span> (packet == <span class="pl-c1">0</span>) packet = dmpPacketBuffer;</td>
      </tr>
      <tr>
        <td id="L548" class="blob-num js-line-number" data-line-number="548"></td>
        <td id="LC548" class="blob-code blob-code-inner js-file-line">    data[<span class="pl-c1">0</span>] = ((packet[<span class="pl-c1">28</span>] &lt;&lt; <span class="pl-c1">24</span>) + (packet[<span class="pl-c1">29</span>] &lt;&lt; <span class="pl-c1">16</span>) + (packet[<span class="pl-c1">30</span>] &lt;&lt; <span class="pl-c1">8</span>) + packet[<span class="pl-c1">31</span>]);</td>
      </tr>
      <tr>
        <td id="L549" class="blob-num js-line-number" data-line-number="549"></td>
        <td id="LC549" class="blob-code blob-code-inner js-file-line">    data[<span class="pl-c1">1</span>] = ((packet[<span class="pl-c1">32</span>] &lt;&lt; <span class="pl-c1">24</span>) + (packet[<span class="pl-c1">33</span>] &lt;&lt; <span class="pl-c1">16</span>) + (packet[<span class="pl-c1">34</span>] &lt;&lt; <span class="pl-c1">8</span>) + packet[<span class="pl-c1">35</span>]);</td>
      </tr>
      <tr>
        <td id="L550" class="blob-num js-line-number" data-line-number="550"></td>
        <td id="LC550" class="blob-code blob-code-inner js-file-line">    data[<span class="pl-c1">2</span>] = ((packet[<span class="pl-c1">36</span>] &lt;&lt; <span class="pl-c1">24</span>) + (packet[<span class="pl-c1">37</span>] &lt;&lt; <span class="pl-c1">16</span>) + (packet[<span class="pl-c1">38</span>] &lt;&lt; <span class="pl-c1">8</span>) + packet[<span class="pl-c1">39</span>]);</td>
      </tr>
      <tr>
        <td id="L551" class="blob-num js-line-number" data-line-number="551"></td>
        <td id="LC551" class="blob-code blob-code-inner js-file-line">    <span class="pl-k">return</span> <span class="pl-c1">0</span>;</td>
      </tr>
      <tr>
        <td id="L552" class="blob-num js-line-number" data-line-number="552"></td>
        <td id="LC552" class="blob-code blob-code-inner js-file-line">}</td>
      </tr>
      <tr>
        <td id="L553" class="blob-num js-line-number" data-line-number="553"></td>
        <td id="LC553" class="blob-code blob-code-inner js-file-line"><span class="pl-c1">uint8_t</span> <span class="pl-en">MPU6050::dmpGetAccel</span>(<span class="pl-c1">int16_t</span> *data, <span class="pl-k">const</span> <span class="pl-c1">uint8_t</span>* packet) {</td>
      </tr>
      <tr>
        <td id="L554" class="blob-num js-line-number" data-line-number="554"></td>
        <td id="LC554" class="blob-code blob-code-inner js-file-line">    <span class="pl-c"><span class="pl-c">//</span> TODO: accommodate different arrangements of sent data (ONLY default supported now)</span></td>
      </tr>
      <tr>
        <td id="L555" class="blob-num js-line-number" data-line-number="555"></td>
        <td id="LC555" class="blob-code blob-code-inner js-file-line">    <span class="pl-k">if</span> (packet == <span class="pl-c1">0</span>) packet = dmpPacketBuffer;</td>
      </tr>
      <tr>
        <td id="L556" class="blob-num js-line-number" data-line-number="556"></td>
        <td id="LC556" class="blob-code blob-code-inner js-file-line">    data[<span class="pl-c1">0</span>] = (packet[<span class="pl-c1">28</span>] &lt;&lt; <span class="pl-c1">8</span>) + packet[<span class="pl-c1">29</span>];</td>
      </tr>
      <tr>
        <td id="L557" class="blob-num js-line-number" data-line-number="557"></td>
        <td id="LC557" class="blob-code blob-code-inner js-file-line">    data[<span class="pl-c1">1</span>] = (packet[<span class="pl-c1">32</span>] &lt;&lt; <span class="pl-c1">8</span>) + packet[<span class="pl-c1">33</span>];</td>
      </tr>
      <tr>
        <td id="L558" class="blob-num js-line-number" data-line-number="558"></td>
        <td id="LC558" class="blob-code blob-code-inner js-file-line">    data[<span class="pl-c1">2</span>] = (packet[<span class="pl-c1">36</span>] &lt;&lt; <span class="pl-c1">8</span>) + packet[<span class="pl-c1">37</span>];</td>
      </tr>
      <tr>
        <td id="L559" class="blob-num js-line-number" data-line-number="559"></td>
        <td id="LC559" class="blob-code blob-code-inner js-file-line">    <span class="pl-k">return</span> <span class="pl-c1">0</span>;</td>
      </tr>
      <tr>
        <td id="L560" class="blob-num js-line-number" data-line-number="560"></td>
        <td id="LC560" class="blob-code blob-code-inner js-file-line">}</td>
      </tr>
      <tr>
        <td id="L561" class="blob-num js-line-number" data-line-number="561"></td>
        <td id="LC561" class="blob-code blob-code-inner js-file-line"><span class="pl-c1">uint8_t</span> <span class="pl-en">MPU6050::dmpGetAccel</span>(VectorInt16 *v, <span class="pl-k">const</span> <span class="pl-c1">uint8_t</span>* packet) {</td>
      </tr>
      <tr>
        <td id="L562" class="blob-num js-line-number" data-line-number="562"></td>
        <td id="LC562" class="blob-code blob-code-inner js-file-line">    <span class="pl-c"><span class="pl-c">//</span> TODO: accommodate different arrangements of sent data (ONLY default supported now)</span></td>
      </tr>
      <tr>
        <td id="L563" class="blob-num js-line-number" data-line-number="563"></td>
        <td id="LC563" class="blob-code blob-code-inner js-file-line">    <span class="pl-k">if</span> (packet == <span class="pl-c1">0</span>) packet = dmpPacketBuffer;</td>
      </tr>
      <tr>
        <td id="L564" class="blob-num js-line-number" data-line-number="564"></td>
        <td id="LC564" class="blob-code blob-code-inner js-file-line">    v -&gt; x = (packet[<span class="pl-c1">28</span>] &lt;&lt; <span class="pl-c1">8</span>) + packet[<span class="pl-c1">29</span>];</td>
      </tr>
      <tr>
        <td id="L565" class="blob-num js-line-number" data-line-number="565"></td>
        <td id="LC565" class="blob-code blob-code-inner js-file-line">    v -&gt; y = (packet[<span class="pl-c1">32</span>] &lt;&lt; <span class="pl-c1">8</span>) + packet[<span class="pl-c1">33</span>];</td>
      </tr>
      <tr>
        <td id="L566" class="blob-num js-line-number" data-line-number="566"></td>
        <td id="LC566" class="blob-code blob-code-inner js-file-line">    v -&gt; z = (packet[<span class="pl-c1">36</span>] &lt;&lt; <span class="pl-c1">8</span>) + packet[<span class="pl-c1">37</span>];</td>
      </tr>
      <tr>
        <td id="L567" class="blob-num js-line-number" data-line-number="567"></td>
        <td id="LC567" class="blob-code blob-code-inner js-file-line">    <span class="pl-k">return</span> <span class="pl-c1">0</span>;</td>
      </tr>
      <tr>
        <td id="L568" class="blob-num js-line-number" data-line-number="568"></td>
        <td id="LC568" class="blob-code blob-code-inner js-file-line">}</td>
      </tr>
      <tr>
        <td id="L569" class="blob-num js-line-number" data-line-number="569"></td>
        <td id="LC569" class="blob-code blob-code-inner js-file-line"><span class="pl-c1">uint8_t</span> <span class="pl-en">MPU6050::dmpGetQuaternion</span>(<span class="pl-c1">int32_t</span> *data, <span class="pl-k">const</span> <span class="pl-c1">uint8_t</span>* packet) {</td>
      </tr>
      <tr>
        <td id="L570" class="blob-num js-line-number" data-line-number="570"></td>
        <td id="LC570" class="blob-code blob-code-inner js-file-line">    <span class="pl-c"><span class="pl-c">//</span> TODO: accommodate different arrangements of sent data (ONLY default supported now)</span></td>
      </tr>
      <tr>
        <td id="L571" class="blob-num js-line-number" data-line-number="571"></td>
        <td id="LC571" class="blob-code blob-code-inner js-file-line">    <span class="pl-k">if</span> (packet == <span class="pl-c1">0</span>) packet = dmpPacketBuffer;</td>
      </tr>
      <tr>
        <td id="L572" class="blob-num js-line-number" data-line-number="572"></td>
        <td id="LC572" class="blob-code blob-code-inner js-file-line">    data[<span class="pl-c1">0</span>] = ((packet[<span class="pl-c1">0</span>] &lt;&lt; <span class="pl-c1">24</span>) + (packet[<span class="pl-c1">1</span>] &lt;&lt; <span class="pl-c1">16</span>) + (packet[<span class="pl-c1">2</span>] &lt;&lt; <span class="pl-c1">8</span>) + packet[<span class="pl-c1">3</span>]);</td>
      </tr>
      <tr>
        <td id="L573" class="blob-num js-line-number" data-line-number="573"></td>
        <td id="LC573" class="blob-code blob-code-inner js-file-line">    data[<span class="pl-c1">1</span>] = ((packet[<span class="pl-c1">4</span>] &lt;&lt; <span class="pl-c1">24</span>) + (packet[<span class="pl-c1">5</span>] &lt;&lt; <span class="pl-c1">16</span>) + (packet[<span class="pl-c1">6</span>] &lt;&lt; <span class="pl-c1">8</span>) + packet[<span class="pl-c1">7</span>]);</td>
      </tr>
      <tr>
        <td id="L574" class="blob-num js-line-number" data-line-number="574"></td>
        <td id="LC574" class="blob-code blob-code-inner js-file-line">    data[<span class="pl-c1">2</span>] = ((packet[<span class="pl-c1">8</span>] &lt;&lt; <span class="pl-c1">24</span>) + (packet[<span class="pl-c1">9</span>] &lt;&lt; <span class="pl-c1">16</span>) + (packet[<span class="pl-c1">10</span>] &lt;&lt; <span class="pl-c1">8</span>) + packet[<span class="pl-c1">11</span>]);</td>
      </tr>
      <tr>
        <td id="L575" class="blob-num js-line-number" data-line-number="575"></td>
        <td id="LC575" class="blob-code blob-code-inner js-file-line">    data[<span class="pl-c1">3</span>] = ((packet[<span class="pl-c1">12</span>] &lt;&lt; <span class="pl-c1">24</span>) + (packet[<span class="pl-c1">13</span>] &lt;&lt; <span class="pl-c1">16</span>) + (packet[<span class="pl-c1">14</span>] &lt;&lt; <span class="pl-c1">8</span>) + packet[<span class="pl-c1">15</span>]);</td>
      </tr>
      <tr>
        <td id="L576" class="blob-num js-line-number" data-line-number="576"></td>
        <td id="LC576" class="blob-code blob-code-inner js-file-line">    <span class="pl-k">return</span> <span class="pl-c1">0</span>;</td>
      </tr>
      <tr>
        <td id="L577" class="blob-num js-line-number" data-line-number="577"></td>
        <td id="LC577" class="blob-code blob-code-inner js-file-line">}</td>
      </tr>
      <tr>
        <td id="L578" class="blob-num js-line-number" data-line-number="578"></td>
        <td id="LC578" class="blob-code blob-code-inner js-file-line"><span class="pl-c1">uint8_t</span> <span class="pl-en">MPU6050::dmpGetQuaternion</span>(<span class="pl-c1">int16_t</span> *data, <span class="pl-k">const</span> <span class="pl-c1">uint8_t</span>* packet) {</td>
      </tr>
      <tr>
        <td id="L579" class="blob-num js-line-number" data-line-number="579"></td>
        <td id="LC579" class="blob-code blob-code-inner js-file-line">    <span class="pl-c"><span class="pl-c">//</span> TODO: accommodate different arrangements of sent data (ONLY default supported now)</span></td>
      </tr>
      <tr>
        <td id="L580" class="blob-num js-line-number" data-line-number="580"></td>
        <td id="LC580" class="blob-code blob-code-inner js-file-line">    <span class="pl-k">if</span> (packet == <span class="pl-c1">0</span>) packet = dmpPacketBuffer;</td>
      </tr>
      <tr>
        <td id="L581" class="blob-num js-line-number" data-line-number="581"></td>
        <td id="LC581" class="blob-code blob-code-inner js-file-line">    data[<span class="pl-c1">0</span>] = ((packet[<span class="pl-c1">0</span>] &lt;&lt; <span class="pl-c1">8</span>) + packet[<span class="pl-c1">1</span>]);</td>
      </tr>
      <tr>
        <td id="L582" class="blob-num js-line-number" data-line-number="582"></td>
        <td id="LC582" class="blob-code blob-code-inner js-file-line">    data[<span class="pl-c1">1</span>] = ((packet[<span class="pl-c1">4</span>] &lt;&lt; <span class="pl-c1">8</span>) + packet[<span class="pl-c1">5</span>]);</td>
      </tr>
      <tr>
        <td id="L583" class="blob-num js-line-number" data-line-number="583"></td>
        <td id="LC583" class="blob-code blob-code-inner js-file-line">    data[<span class="pl-c1">2</span>] = ((packet[<span class="pl-c1">8</span>] &lt;&lt; <span class="pl-c1">8</span>) + packet[<span class="pl-c1">9</span>]);</td>
      </tr>
      <tr>
        <td id="L584" class="blob-num js-line-number" data-line-number="584"></td>
        <td id="LC584" class="blob-code blob-code-inner js-file-line">    data[<span class="pl-c1">3</span>] = ((packet[<span class="pl-c1">12</span>] &lt;&lt; <span class="pl-c1">8</span>) + packet[<span class="pl-c1">13</span>]);</td>
      </tr>
      <tr>
        <td id="L585" class="blob-num js-line-number" data-line-number="585"></td>
        <td id="LC585" class="blob-code blob-code-inner js-file-line">    <span class="pl-k">return</span> <span class="pl-c1">0</span>;</td>
      </tr>
      <tr>
        <td id="L586" class="blob-num js-line-number" data-line-number="586"></td>
        <td id="LC586" class="blob-code blob-code-inner js-file-line">}</td>
      </tr>
      <tr>
        <td id="L587" class="blob-num js-line-number" data-line-number="587"></td>
        <td id="LC587" class="blob-code blob-code-inner js-file-line"><span class="pl-c1">uint8_t</span> <span class="pl-en">MPU6050::dmpGetQuaternion</span>(Quaternion *q, <span class="pl-k">const</span> <span class="pl-c1">uint8_t</span>* packet) {</td>
      </tr>
      <tr>
        <td id="L588" class="blob-num js-line-number" data-line-number="588"></td>
        <td id="LC588" class="blob-code blob-code-inner js-file-line">    <span class="pl-c"><span class="pl-c">//</span> TODO: accommodate different arrangements of sent data (ONLY default supported now)</span></td>
      </tr>
      <tr>
        <td id="L589" class="blob-num js-line-number" data-line-number="589"></td>
        <td id="LC589" class="blob-code blob-code-inner js-file-line">    <span class="pl-c1">int16_t</span> qI[<span class="pl-c1">4</span>];</td>
      </tr>
      <tr>
        <td id="L590" class="blob-num js-line-number" data-line-number="590"></td>
        <td id="LC590" class="blob-code blob-code-inner js-file-line">    <span class="pl-c1">uint8_t</span> status = <span class="pl-c1">dmpGetQuaternion</span>(qI, packet);</td>
      </tr>
      <tr>
        <td id="L591" class="blob-num js-line-number" data-line-number="591"></td>
        <td id="LC591" class="blob-code blob-code-inner js-file-line">    <span class="pl-k">if</span> (status == <span class="pl-c1">0</span>) {</td>
      </tr>
      <tr>
        <td id="L592" class="blob-num js-line-number" data-line-number="592"></td>
        <td id="LC592" class="blob-code blob-code-inner js-file-line">        q -&gt; w = (<span class="pl-k">float</span>)qI[<span class="pl-c1">0</span>] / <span class="pl-c1">16384</span>.<span class="pl-c1">0f</span>;</td>
      </tr>
      <tr>
        <td id="L593" class="blob-num js-line-number" data-line-number="593"></td>
        <td id="LC593" class="blob-code blob-code-inner js-file-line">        q -&gt; x = (<span class="pl-k">float</span>)qI[<span class="pl-c1">1</span>] / <span class="pl-c1">16384</span>.<span class="pl-c1">0f</span>;</td>
      </tr>
      <tr>
        <td id="L594" class="blob-num js-line-number" data-line-number="594"></td>
        <td id="LC594" class="blob-code blob-code-inner js-file-line">        q -&gt; y = (<span class="pl-k">float</span>)qI[<span class="pl-c1">2</span>] / <span class="pl-c1">16384</span>.<span class="pl-c1">0f</span>;</td>
      </tr>
      <tr>
        <td id="L595" class="blob-num js-line-number" data-line-number="595"></td>
        <td id="LC595" class="blob-code blob-code-inner js-file-line">        q -&gt; z = (<span class="pl-k">float</span>)qI[<span class="pl-c1">3</span>] / <span class="pl-c1">16384</span>.<span class="pl-c1">0f</span>;</td>
      </tr>
      <tr>
        <td id="L596" class="blob-num js-line-number" data-line-number="596"></td>
        <td id="LC596" class="blob-code blob-code-inner js-file-line">        <span class="pl-k">return</span> <span class="pl-c1">0</span>;</td>
      </tr>
      <tr>
        <td id="L597" class="blob-num js-line-number" data-line-number="597"></td>
        <td id="LC597" class="blob-code blob-code-inner js-file-line">    }</td>
      </tr>
      <tr>
        <td id="L598" class="blob-num js-line-number" data-line-number="598"></td>
        <td id="LC598" class="blob-code blob-code-inner js-file-line">    <span class="pl-k">return</span> status; <span class="pl-c"><span class="pl-c">//</span> int16 return value, indicates error if this line is reached</span></td>
      </tr>
      <tr>
        <td id="L599" class="blob-num js-line-number" data-line-number="599"></td>
        <td id="LC599" class="blob-code blob-code-inner js-file-line">}</td>
      </tr>
      <tr>
        <td id="L600" class="blob-num js-line-number" data-line-number="600"></td>
        <td id="LC600" class="blob-code blob-code-inner js-file-line"><span class="pl-c"><span class="pl-c">//</span> uint8_t MPU6050::dmpGet6AxisQuaternion(long *data, const uint8_t* packet);</span></td>
      </tr>
      <tr>
        <td id="L601" class="blob-num js-line-number" data-line-number="601"></td>
        <td id="LC601" class="blob-code blob-code-inner js-file-line"><span class="pl-c"><span class="pl-c">//</span> uint8_t MPU6050::dmpGetRelativeQuaternion(long *data, const uint8_t* packet);</span></td>
      </tr>
      <tr>
        <td id="L602" class="blob-num js-line-number" data-line-number="602"></td>
        <td id="LC602" class="blob-code blob-code-inner js-file-line"><span class="pl-c1">uint8_t</span> <span class="pl-en">MPU6050::dmpGetGyro</span>(<span class="pl-c1">int32_t</span> *data, <span class="pl-k">const</span> <span class="pl-c1">uint8_t</span>* packet) {</td>
      </tr>
      <tr>
        <td id="L603" class="blob-num js-line-number" data-line-number="603"></td>
        <td id="LC603" class="blob-code blob-code-inner js-file-line">    <span class="pl-c"><span class="pl-c">//</span> TODO: accommodate different arrangements of sent data (ONLY default supported now)</span></td>
      </tr>
      <tr>
        <td id="L604" class="blob-num js-line-number" data-line-number="604"></td>
        <td id="LC604" class="blob-code blob-code-inner js-file-line">    <span class="pl-k">if</span> (packet == <span class="pl-c1">0</span>) packet = dmpPacketBuffer;</td>
      </tr>
      <tr>
        <td id="L605" class="blob-num js-line-number" data-line-number="605"></td>
        <td id="LC605" class="blob-code blob-code-inner js-file-line">    data[<span class="pl-c1">0</span>] = ((packet[<span class="pl-c1">16</span>] &lt;&lt; <span class="pl-c1">24</span>) + (packet[<span class="pl-c1">17</span>] &lt;&lt; <span class="pl-c1">16</span>) + (packet[<span class="pl-c1">18</span>] &lt;&lt; <span class="pl-c1">8</span>) + packet[<span class="pl-c1">19</span>]);</td>
      </tr>
      <tr>
        <td id="L606" class="blob-num js-line-number" data-line-number="606"></td>
        <td id="LC606" class="blob-code blob-code-inner js-file-line">    data[<span class="pl-c1">1</span>] = ((packet[<span class="pl-c1">20</span>] &lt;&lt; <span class="pl-c1">24</span>) + (packet[<span class="pl-c1">21</span>] &lt;&lt; <span class="pl-c1">16</span>) + (packet[<span class="pl-c1">22</span>] &lt;&lt; <span class="pl-c1">8</span>) + packet[<span class="pl-c1">23</span>]);</td>
      </tr>
      <tr>
        <td id="L607" class="blob-num js-line-number" data-line-number="607"></td>
        <td id="LC607" class="blob-code blob-code-inner js-file-line">    data[<span class="pl-c1">2</span>] = ((packet[<span class="pl-c1">24</span>] &lt;&lt; <span class="pl-c1">24</span>) + (packet[<span class="pl-c1">25</span>] &lt;&lt; <span class="pl-c1">16</span>) + (packet[<span class="pl-c1">26</span>] &lt;&lt; <span class="pl-c1">8</span>) + packet[<span class="pl-c1">27</span>]);</td>
      </tr>
      <tr>
        <td id="L608" class="blob-num js-line-number" data-line-number="608"></td>
        <td id="LC608" class="blob-code blob-code-inner js-file-line">    <span class="pl-k">return</span> <span class="pl-c1">0</span>;</td>
      </tr>
      <tr>
        <td id="L609" class="blob-num js-line-number" data-line-number="609"></td>
        <td id="LC609" class="blob-code blob-code-inner js-file-line">}</td>
      </tr>
      <tr>
        <td id="L610" class="blob-num js-line-number" data-line-number="610"></td>
        <td id="LC610" class="blob-code blob-code-inner js-file-line"><span class="pl-c1">uint8_t</span> <span class="pl-en">MPU6050::dmpGetGyro</span>(<span class="pl-c1">int16_t</span> *data, <span class="pl-k">const</span> <span class="pl-c1">uint8_t</span>* packet) {</td>
      </tr>
      <tr>
        <td id="L611" class="blob-num js-line-number" data-line-number="611"></td>
        <td id="LC611" class="blob-code blob-code-inner js-file-line">    <span class="pl-c"><span class="pl-c">//</span> TODO: accommodate different arrangements of sent data (ONLY default supported now)</span></td>
      </tr>
      <tr>
        <td id="L612" class="blob-num js-line-number" data-line-number="612"></td>
        <td id="LC612" class="blob-code blob-code-inner js-file-line">    <span class="pl-k">if</span> (packet == <span class="pl-c1">0</span>) packet = dmpPacketBuffer;</td>
      </tr>
      <tr>
        <td id="L613" class="blob-num js-line-number" data-line-number="613"></td>
        <td id="LC613" class="blob-code blob-code-inner js-file-line">    data[<span class="pl-c1">0</span>] = (packet[<span class="pl-c1">16</span>] &lt;&lt; <span class="pl-c1">8</span>) + packet[<span class="pl-c1">17</span>];</td>
      </tr>
      <tr>
        <td id="L614" class="blob-num js-line-number" data-line-number="614"></td>
        <td id="LC614" class="blob-code blob-code-inner js-file-line">    data[<span class="pl-c1">1</span>] = (packet[<span class="pl-c1">20</span>] &lt;&lt; <span class="pl-c1">8</span>) + packet[<span class="pl-c1">21</span>];</td>
      </tr>
      <tr>
        <td id="L615" class="blob-num js-line-number" data-line-number="615"></td>
        <td id="LC615" class="blob-code blob-code-inner js-file-line">    data[<span class="pl-c1">2</span>] = (packet[<span class="pl-c1">24</span>] &lt;&lt; <span class="pl-c1">8</span>) + packet[<span class="pl-c1">25</span>];</td>
      </tr>
      <tr>
        <td id="L616" class="blob-num js-line-number" data-line-number="616"></td>
        <td id="LC616" class="blob-code blob-code-inner js-file-line">    <span class="pl-k">return</span> <span class="pl-c1">0</span>;</td>
      </tr>
      <tr>
        <td id="L617" class="blob-num js-line-number" data-line-number="617"></td>
        <td id="LC617" class="blob-code blob-code-inner js-file-line">}</td>
      </tr>
      <tr>
        <td id="L618" class="blob-num js-line-number" data-line-number="618"></td>
        <td id="LC618" class="blob-code blob-code-inner js-file-line"><span class="pl-c"><span class="pl-c">//</span> uint8_t MPU6050::dmpSetLinearAccelFilterCoefficient(float coef);</span></td>
      </tr>
      <tr>
        <td id="L619" class="blob-num js-line-number" data-line-number="619"></td>
        <td id="LC619" class="blob-code blob-code-inner js-file-line"><span class="pl-c"><span class="pl-c">//</span> uint8_t MPU6050::dmpGetLinearAccel(long *data, const uint8_t* packet);</span></td>
      </tr>
      <tr>
        <td id="L620" class="blob-num js-line-number" data-line-number="620"></td>
        <td id="LC620" class="blob-code blob-code-inner js-file-line"><span class="pl-c1">uint8_t</span> <span class="pl-en">MPU6050::dmpGetLinearAccel</span>(VectorInt16 *v, VectorInt16 *vRaw, VectorFloat *gravity) {</td>
      </tr>
      <tr>
        <td id="L621" class="blob-num js-line-number" data-line-number="621"></td>
        <td id="LC621" class="blob-code blob-code-inner js-file-line">    <span class="pl-c"><span class="pl-c">//</span> get rid of the gravity component (+1g = +4096 in standard DMP FIFO packet)</span></td>
      </tr>
      <tr>
        <td id="L622" class="blob-num js-line-number" data-line-number="622"></td>
        <td id="LC622" class="blob-code blob-code-inner js-file-line">    v -&gt; x = vRaw -&gt; x - gravity -&gt; x*<span class="pl-c1">4096</span>;</td>
      </tr>
      <tr>
        <td id="L623" class="blob-num js-line-number" data-line-number="623"></td>
        <td id="LC623" class="blob-code blob-code-inner js-file-line">    v -&gt; y = vRaw -&gt; y - gravity -&gt; y*<span class="pl-c1">4096</span>;</td>
      </tr>
      <tr>
        <td id="L624" class="blob-num js-line-number" data-line-number="624"></td>
        <td id="LC624" class="blob-code blob-code-inner js-file-line">    v -&gt; z = vRaw -&gt; z - gravity -&gt; z*<span class="pl-c1">4096</span>;</td>
      </tr>
      <tr>
        <td id="L625" class="blob-num js-line-number" data-line-number="625"></td>
        <td id="LC625" class="blob-code blob-code-inner js-file-line">    <span class="pl-k">return</span> <span class="pl-c1">0</span>;</td>
      </tr>
      <tr>
        <td id="L626" class="blob-num js-line-number" data-line-number="626"></td>
        <td id="LC626" class="blob-code blob-code-inner js-file-line">}</td>
      </tr>
      <tr>
        <td id="L627" class="blob-num js-line-number" data-line-number="627"></td>
        <td id="LC627" class="blob-code blob-code-inner js-file-line"><span class="pl-c"><span class="pl-c">//</span> uint8_t MPU6050::dmpGetLinearAccelInWorld(long *data, const uint8_t* packet);</span></td>
      </tr>
      <tr>
        <td id="L628" class="blob-num js-line-number" data-line-number="628"></td>
        <td id="LC628" class="blob-code blob-code-inner js-file-line"><span class="pl-c1">uint8_t</span> <span class="pl-en">MPU6050::dmpGetLinearAccelInWorld</span>(VectorInt16 *v, VectorInt16 *vReal, Quaternion *q) {</td>
      </tr>
      <tr>
        <td id="L629" class="blob-num js-line-number" data-line-number="629"></td>
        <td id="LC629" class="blob-code blob-code-inner js-file-line">    <span class="pl-c"><span class="pl-c">//</span> rotate measured 3D acceleration vector into original state</span></td>
      </tr>
      <tr>
        <td id="L630" class="blob-num js-line-number" data-line-number="630"></td>
        <td id="LC630" class="blob-code blob-code-inner js-file-line">    <span class="pl-c"><span class="pl-c">//</span> frame of reference based on orientation quaternion</span></td>
      </tr>
      <tr>
        <td id="L631" class="blob-num js-line-number" data-line-number="631"></td>
        <td id="LC631" class="blob-code blob-code-inner js-file-line">    <span class="pl-c1">memcpy</span>(v, vReal, <span class="pl-k">sizeof</span>(VectorInt16));</td>
      </tr>
      <tr>
        <td id="L632" class="blob-num js-line-number" data-line-number="632"></td>
        <td id="LC632" class="blob-code blob-code-inner js-file-line">    v -&gt; <span class="pl-c1">rotate</span>(q);</td>
      </tr>
      <tr>
        <td id="L633" class="blob-num js-line-number" data-line-number="633"></td>
        <td id="LC633" class="blob-code blob-code-inner js-file-line">    <span class="pl-k">return</span> <span class="pl-c1">0</span>;</td>
      </tr>
      <tr>
        <td id="L634" class="blob-num js-line-number" data-line-number="634"></td>
        <td id="LC634" class="blob-code blob-code-inner js-file-line">}</td>
      </tr>
      <tr>
        <td id="L635" class="blob-num js-line-number" data-line-number="635"></td>
        <td id="LC635" class="blob-code blob-code-inner js-file-line"><span class="pl-c"><span class="pl-c">//</span> uint8_t MPU6050::dmpGetGyroAndAccelSensor(long *data, const uint8_t* packet);</span></td>
      </tr>
      <tr>
        <td id="L636" class="blob-num js-line-number" data-line-number="636"></td>
        <td id="LC636" class="blob-code blob-code-inner js-file-line"><span class="pl-c"><span class="pl-c">//</span> uint8_t MPU6050::dmpGetGyroSensor(long *data, const uint8_t* packet);</span></td>
      </tr>
      <tr>
        <td id="L637" class="blob-num js-line-number" data-line-number="637"></td>
        <td id="LC637" class="blob-code blob-code-inner js-file-line"><span class="pl-c"><span class="pl-c">//</span> uint8_t MPU6050::dmpGetControlData(long *data, const uint8_t* packet);</span></td>
      </tr>
      <tr>
        <td id="L638" class="blob-num js-line-number" data-line-number="638"></td>
        <td id="LC638" class="blob-code blob-code-inner js-file-line"><span class="pl-c"><span class="pl-c">//</span> uint8_t MPU6050::dmpGetTemperature(long *data, const uint8_t* packet);</span></td>
      </tr>
      <tr>
        <td id="L639" class="blob-num js-line-number" data-line-number="639"></td>
        <td id="LC639" class="blob-code blob-code-inner js-file-line"><span class="pl-c"><span class="pl-c">//</span> uint8_t MPU6050::dmpGetGravity(long *data, const uint8_t* packet);</span></td>
      </tr>
      <tr>
        <td id="L640" class="blob-num js-line-number" data-line-number="640"></td>
        <td id="LC640" class="blob-code blob-code-inner js-file-line"><span class="pl-c1">uint8_t</span> <span class="pl-en">MPU6050::dmpGetGravity</span>(VectorFloat *v, Quaternion *q) {</td>
      </tr>
      <tr>
        <td id="L641" class="blob-num js-line-number" data-line-number="641"></td>
        <td id="LC641" class="blob-code blob-code-inner js-file-line">    v -&gt; x = <span class="pl-c1">2</span> * (q -&gt; x*q -&gt; z - q -&gt; w*q -&gt; y);</td>
      </tr>
      <tr>
        <td id="L642" class="blob-num js-line-number" data-line-number="642"></td>
        <td id="LC642" class="blob-code blob-code-inner js-file-line">    v -&gt; y = <span class="pl-c1">2</span> * (q -&gt; w*q -&gt; x + q -&gt; y*q -&gt; z);</td>
      </tr>
      <tr>
        <td id="L643" class="blob-num js-line-number" data-line-number="643"></td>
        <td id="LC643" class="blob-code blob-code-inner js-file-line">    v -&gt; z = q -&gt; w*q -&gt; w - q -&gt; x*q -&gt; x - q -&gt; y*q -&gt; y + q -&gt; z*q -&gt; z;</td>
      </tr>
      <tr>
        <td id="L644" class="blob-num js-line-number" data-line-number="644"></td>
        <td id="LC644" class="blob-code blob-code-inner js-file-line">    <span class="pl-k">return</span> <span class="pl-c1">0</span>;</td>
      </tr>
      <tr>
        <td id="L645" class="blob-num js-line-number" data-line-number="645"></td>
        <td id="LC645" class="blob-code blob-code-inner js-file-line">}</td>
      </tr>
      <tr>
        <td id="L646" class="blob-num js-line-number" data-line-number="646"></td>
        <td id="LC646" class="blob-code blob-code-inner js-file-line"><span class="pl-c"><span class="pl-c">//</span> uint8_t MPU6050::dmpGetUnquantizedAccel(long *data, const uint8_t* packet);</span></td>
      </tr>
      <tr>
        <td id="L647" class="blob-num js-line-number" data-line-number="647"></td>
        <td id="LC647" class="blob-code blob-code-inner js-file-line"><span class="pl-c"><span class="pl-c">//</span> uint8_t MPU6050::dmpGetQuantizedAccel(long *data, const uint8_t* packet);</span></td>
      </tr>
      <tr>
        <td id="L648" class="blob-num js-line-number" data-line-number="648"></td>
        <td id="LC648" class="blob-code blob-code-inner js-file-line"><span class="pl-c"><span class="pl-c">//</span> uint8_t MPU6050::dmpGetExternalSensorData(long *data, int size, const uint8_t* packet);</span></td>
      </tr>
      <tr>
        <td id="L649" class="blob-num js-line-number" data-line-number="649"></td>
        <td id="LC649" class="blob-code blob-code-inner js-file-line"><span class="pl-c"><span class="pl-c">//</span> uint8_t MPU6050::dmpGetEIS(long *data, const uint8_t* packet);</span></td>
      </tr>
      <tr>
        <td id="L650" class="blob-num js-line-number" data-line-number="650"></td>
        <td id="LC650" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L651" class="blob-num js-line-number" data-line-number="651"></td>
        <td id="LC651" class="blob-code blob-code-inner js-file-line"><span class="pl-c1">uint8_t</span> <span class="pl-en">MPU6050::dmpGetEuler</span>(<span class="pl-k">float</span> *data, Quaternion *q) {</td>
      </tr>
      <tr>
        <td id="L652" class="blob-num js-line-number" data-line-number="652"></td>
        <td id="LC652" class="blob-code blob-code-inner js-file-line">    data[<span class="pl-c1">0</span>] = <span class="pl-c1">atan2</span>(<span class="pl-c1">2</span>*q -&gt; x*q -&gt; y - <span class="pl-c1">2</span>*q -&gt; w*q -&gt; z, <span class="pl-c1">2</span>*q -&gt; w*q -&gt; w + <span class="pl-c1">2</span>*q -&gt; x*q -&gt; x - <span class="pl-c1">1</span>);   <span class="pl-c"><span class="pl-c">//</span> psi</span></td>
      </tr>
      <tr>
        <td id="L653" class="blob-num js-line-number" data-line-number="653"></td>
        <td id="LC653" class="blob-code blob-code-inner js-file-line">    data[<span class="pl-c1">1</span>] = -<span class="pl-c1">asin</span>(<span class="pl-c1">2</span>*q -&gt; x*q -&gt; z + <span class="pl-c1">2</span>*q -&gt; w*q -&gt; y);                              <span class="pl-c"><span class="pl-c">//</span> theta</span></td>
      </tr>
      <tr>
        <td id="L654" class="blob-num js-line-number" data-line-number="654"></td>
        <td id="LC654" class="blob-code blob-code-inner js-file-line">    data[<span class="pl-c1">2</span>] = <span class="pl-c1">atan2</span>(<span class="pl-c1">2</span>*q -&gt; y*q -&gt; z - <span class="pl-c1">2</span>*q -&gt; w*q -&gt; x, <span class="pl-c1">2</span>*q -&gt; w*q -&gt; w + <span class="pl-c1">2</span>*q -&gt; z*q -&gt; z - <span class="pl-c1">1</span>);   <span class="pl-c"><span class="pl-c">//</span> phi</span></td>
      </tr>
      <tr>
        <td id="L655" class="blob-num js-line-number" data-line-number="655"></td>
        <td id="LC655" class="blob-code blob-code-inner js-file-line">    <span class="pl-k">return</span> <span class="pl-c1">0</span>;</td>
      </tr>
      <tr>
        <td id="L656" class="blob-num js-line-number" data-line-number="656"></td>
        <td id="LC656" class="blob-code blob-code-inner js-file-line">}</td>
      </tr>
      <tr>
        <td id="L657" class="blob-num js-line-number" data-line-number="657"></td>
        <td id="LC657" class="blob-code blob-code-inner js-file-line"><span class="pl-c1">uint8_t</span> <span class="pl-en">MPU6050::dmpGetYawPitchRoll</span>(<span class="pl-k">float</span> *data, Quaternion *q, VectorFloat *gravity) {</td>
      </tr>
      <tr>
        <td id="L658" class="blob-num js-line-number" data-line-number="658"></td>
        <td id="LC658" class="blob-code blob-code-inner js-file-line">    <span class="pl-c"><span class="pl-c">//</span> yaw: (about Z axis)</span></td>
      </tr>
      <tr>
        <td id="L659" class="blob-num js-line-number" data-line-number="659"></td>
        <td id="LC659" class="blob-code blob-code-inner js-file-line">    data[<span class="pl-c1">0</span>] = <span class="pl-c1">atan2</span>(<span class="pl-c1">2</span>*q -&gt; x*q -&gt; y - <span class="pl-c1">2</span>*q -&gt; w*q -&gt; z, <span class="pl-c1">2</span>*q -&gt; w*q -&gt; w + <span class="pl-c1">2</span>*q -&gt; x*q -&gt; x - <span class="pl-c1">1</span>);</td>
      </tr>
      <tr>
        <td id="L660" class="blob-num js-line-number" data-line-number="660"></td>
        <td id="LC660" class="blob-code blob-code-inner js-file-line">    <span class="pl-c"><span class="pl-c">//</span> pitch: (nose up/down, about Y axis)</span></td>
      </tr>
      <tr>
        <td id="L661" class="blob-num js-line-number" data-line-number="661"></td>
        <td id="LC661" class="blob-code blob-code-inner js-file-line">    data[<span class="pl-c1">1</span>] = <span class="pl-c1">atan</span>(gravity -&gt; x / <span class="pl-c1">sqrt</span>(gravity -&gt; y*gravity -&gt; y + gravity -&gt; z*gravity -&gt; z));</td>
      </tr>
      <tr>
        <td id="L662" class="blob-num js-line-number" data-line-number="662"></td>
        <td id="LC662" class="blob-code blob-code-inner js-file-line">    <span class="pl-c"><span class="pl-c">//</span> roll: (tilt left/right, about X axis)</span></td>
      </tr>
      <tr>
        <td id="L663" class="blob-num js-line-number" data-line-number="663"></td>
        <td id="LC663" class="blob-code blob-code-inner js-file-line">    data[<span class="pl-c1">2</span>] = <span class="pl-c1">atan</span>(gravity -&gt; y / <span class="pl-c1">sqrt</span>(gravity -&gt; x*gravity -&gt; x + gravity -&gt; z*gravity -&gt; z));</td>
      </tr>
      <tr>
        <td id="L664" class="blob-num js-line-number" data-line-number="664"></td>
        <td id="LC664" class="blob-code blob-code-inner js-file-line">    <span class="pl-k">return</span> <span class="pl-c1">0</span>;</td>
      </tr>
      <tr>
        <td id="L665" class="blob-num js-line-number" data-line-number="665"></td>
        <td id="LC665" class="blob-code blob-code-inner js-file-line">}</td>
      </tr>
      <tr>
        <td id="L666" class="blob-num js-line-number" data-line-number="666"></td>
        <td id="LC666" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L667" class="blob-num js-line-number" data-line-number="667"></td>
        <td id="LC667" class="blob-code blob-code-inner js-file-line"><span class="pl-c"><span class="pl-c">//</span> uint8_t MPU6050::dmpGetAccelFloat(float *data, const uint8_t* packet);</span></td>
      </tr>
      <tr>
        <td id="L668" class="blob-num js-line-number" data-line-number="668"></td>
        <td id="LC668" class="blob-code blob-code-inner js-file-line"><span class="pl-c"><span class="pl-c">//</span> uint8_t MPU6050::dmpGetQuaternionFloat(float *data, const uint8_t* packet);</span></td>
      </tr>
      <tr>
        <td id="L669" class="blob-num js-line-number" data-line-number="669"></td>
        <td id="LC669" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L670" class="blob-num js-line-number" data-line-number="670"></td>
        <td id="LC670" class="blob-code blob-code-inner js-file-line"><span class="pl-c1">uint8_t</span> <span class="pl-en">MPU6050::dmpProcessFIFOPacket</span>(<span class="pl-k">const</span> <span class="pl-k">unsigned</span> <span class="pl-k">char</span> *dmpData) {</td>
      </tr>
      <tr>
        <td id="L671" class="blob-num js-line-number" data-line-number="671"></td>
        <td id="LC671" class="blob-code blob-code-inner js-file-line">    <span class="pl-c"><span class="pl-c">/*</span>for (uint8_t k = 0; k &lt; dmpPacketSize; k++) {</span></td>
      </tr>
      <tr>
        <td id="L672" class="blob-num js-line-number" data-line-number="672"></td>
        <td id="LC672" class="blob-code blob-code-inner js-file-line"><span class="pl-c">        if (dmpData[k] &lt; 0x10) Serial.print(&quot;0&quot;);</span></td>
      </tr>
      <tr>
        <td id="L673" class="blob-num js-line-number" data-line-number="673"></td>
        <td id="LC673" class="blob-code blob-code-inner js-file-line"><span class="pl-c">        Serial.print(dmpData[k], HEX);</span></td>
      </tr>
      <tr>
        <td id="L674" class="blob-num js-line-number" data-line-number="674"></td>
        <td id="LC674" class="blob-code blob-code-inner js-file-line"><span class="pl-c">        Serial.print(&quot; &quot;);</span></td>
      </tr>
      <tr>
        <td id="L675" class="blob-num js-line-number" data-line-number="675"></td>
        <td id="LC675" class="blob-code blob-code-inner js-file-line"><span class="pl-c">    }</span></td>
      </tr>
      <tr>
        <td id="L676" class="blob-num js-line-number" data-line-number="676"></td>
        <td id="LC676" class="blob-code blob-code-inner js-file-line"><span class="pl-c">    Serial.print(&quot;\n&quot;);<span class="pl-c">*/</span></span></td>
      </tr>
      <tr>
        <td id="L677" class="blob-num js-line-number" data-line-number="677"></td>
        <td id="LC677" class="blob-code blob-code-inner js-file-line">    <span class="pl-c"><span class="pl-c">//</span>Serial.println((uint16_t)dmpPacketBuffer);</span></td>
      </tr>
      <tr>
        <td id="L678" class="blob-num js-line-number" data-line-number="678"></td>
        <td id="LC678" class="blob-code blob-code-inner js-file-line">    <span class="pl-k">return</span> <span class="pl-c1">0</span>;</td>
      </tr>
      <tr>
        <td id="L679" class="blob-num js-line-number" data-line-number="679"></td>
        <td id="LC679" class="blob-code blob-code-inner js-file-line">}</td>
      </tr>
      <tr>
        <td id="L680" class="blob-num js-line-number" data-line-number="680"></td>
        <td id="LC680" class="blob-code blob-code-inner js-file-line"><span class="pl-c1">uint8_t</span> <span class="pl-en">MPU6050::dmpReadAndProcessFIFOPacket</span>(<span class="pl-c1">uint8_t</span> numPackets, <span class="pl-c1">uint8_t</span> *processed) {</td>
      </tr>
      <tr>
        <td id="L681" class="blob-num js-line-number" data-line-number="681"></td>
        <td id="LC681" class="blob-code blob-code-inner js-file-line">    <span class="pl-c1">uint8_t</span> status;</td>
      </tr>
      <tr>
        <td id="L682" class="blob-num js-line-number" data-line-number="682"></td>
        <td id="LC682" class="blob-code blob-code-inner js-file-line">    <span class="pl-c1">uint8_t</span> buf[dmpPacketSize];</td>
      </tr>
      <tr>
        <td id="L683" class="blob-num js-line-number" data-line-number="683"></td>
        <td id="LC683" class="blob-code blob-code-inner js-file-line">    <span class="pl-k">for</span> (<span class="pl-c1">uint8_t</span> i = <span class="pl-c1">0</span>; i &lt; numPackets; i++) {</td>
      </tr>
      <tr>
        <td id="L684" class="blob-num js-line-number" data-line-number="684"></td>
        <td id="LC684" class="blob-code blob-code-inner js-file-line">        <span class="pl-c"><span class="pl-c">//</span> read packet from FIFO</span></td>
      </tr>
      <tr>
        <td id="L685" class="blob-num js-line-number" data-line-number="685"></td>
        <td id="LC685" class="blob-code blob-code-inner js-file-line">        <span class="pl-c1">getFIFOBytes</span>(buf, dmpPacketSize);</td>
      </tr>
      <tr>
        <td id="L686" class="blob-num js-line-number" data-line-number="686"></td>
        <td id="LC686" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L687" class="blob-num js-line-number" data-line-number="687"></td>
        <td id="LC687" class="blob-code blob-code-inner js-file-line">        <span class="pl-c"><span class="pl-c">//</span> process packet</span></td>
      </tr>
      <tr>
        <td id="L688" class="blob-num js-line-number" data-line-number="688"></td>
        <td id="LC688" class="blob-code blob-code-inner js-file-line">        <span class="pl-k">if</span> ((status = <span class="pl-c1">dmpProcessFIFOPacket</span>(buf)) &gt; <span class="pl-c1">0</span>) <span class="pl-k">return</span> status;</td>
      </tr>
      <tr>
        <td id="L689" class="blob-num js-line-number" data-line-number="689"></td>
        <td id="LC689" class="blob-code blob-code-inner js-file-line">        </td>
      </tr>
      <tr>
        <td id="L690" class="blob-num js-line-number" data-line-number="690"></td>
        <td id="LC690" class="blob-code blob-code-inner js-file-line">        <span class="pl-c"><span class="pl-c">//</span> increment external process count variable, if supplied</span></td>
      </tr>
      <tr>
        <td id="L691" class="blob-num js-line-number" data-line-number="691"></td>
        <td id="LC691" class="blob-code blob-code-inner js-file-line">        <span class="pl-k">if</span> (processed != <span class="pl-c1">0</span>) (*processed)++;</td>
      </tr>
      <tr>
        <td id="L692" class="blob-num js-line-number" data-line-number="692"></td>
        <td id="LC692" class="blob-code blob-code-inner js-file-line">    }</td>
      </tr>
      <tr>
        <td id="L693" class="blob-num js-line-number" data-line-number="693"></td>
        <td id="LC693" class="blob-code blob-code-inner js-file-line">    <span class="pl-k">return</span> <span class="pl-c1">0</span>;</td>
      </tr>
      <tr>
        <td id="L694" class="blob-num js-line-number" data-line-number="694"></td>
        <td id="LC694" class="blob-code blob-code-inner js-file-line">}</td>
      </tr>
      <tr>
        <td id="L695" class="blob-num js-line-number" data-line-number="695"></td>
        <td id="LC695" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L696" class="blob-num js-line-number" data-line-number="696"></td>
        <td id="LC696" class="blob-code blob-code-inner js-file-line"><span class="pl-c"><span class="pl-c">//</span> uint8_t MPU6050::dmpSetFIFOProcessedCallback(void (*func) (void));</span></td>
      </tr>
      <tr>
        <td id="L697" class="blob-num js-line-number" data-line-number="697"></td>
        <td id="LC697" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L698" class="blob-num js-line-number" data-line-number="698"></td>
        <td id="LC698" class="blob-code blob-code-inner js-file-line"><span class="pl-c"><span class="pl-c">//</span> uint8_t MPU6050::dmpInitFIFOParam();</span></td>
      </tr>
      <tr>
        <td id="L699" class="blob-num js-line-number" data-line-number="699"></td>
        <td id="LC699" class="blob-code blob-code-inner js-file-line"><span class="pl-c"><span class="pl-c">//</span> uint8_t MPU6050::dmpCloseFIFO();</span></td>
      </tr>
      <tr>
        <td id="L700" class="blob-num js-line-number" data-line-number="700"></td>
        <td id="LC700" class="blob-code blob-code-inner js-file-line"><span class="pl-c"><span class="pl-c">//</span> uint8_t MPU6050::dmpSetGyroDataSource(uint_fast8_t source);</span></td>
      </tr>
      <tr>
        <td id="L701" class="blob-num js-line-number" data-line-number="701"></td>
        <td id="LC701" class="blob-code blob-code-inner js-file-line"><span class="pl-c"><span class="pl-c">//</span> uint8_t MPU6050::dmpDecodeQuantizedAccel();</span></td>
      </tr>
      <tr>
        <td id="L702" class="blob-num js-line-number" data-line-number="702"></td>
        <td id="LC702" class="blob-code blob-code-inner js-file-line"><span class="pl-c"><span class="pl-c">//</span> uint32_t MPU6050::dmpGetGyroSumOfSquare();</span></td>
      </tr>
      <tr>
        <td id="L703" class="blob-num js-line-number" data-line-number="703"></td>
        <td id="LC703" class="blob-code blob-code-inner js-file-line"><span class="pl-c"><span class="pl-c">//</span> uint32_t MPU6050::dmpGetAccelSumOfSquare();</span></td>
      </tr>
      <tr>
        <td id="L704" class="blob-num js-line-number" data-line-number="704"></td>
        <td id="LC704" class="blob-code blob-code-inner js-file-line"><span class="pl-c"><span class="pl-c">//</span> void MPU6050::dmpOverrideQuaternion(long *q);</span></td>
      </tr>
      <tr>
        <td id="L705" class="blob-num js-line-number" data-line-number="705"></td>
        <td id="LC705" class="blob-code blob-code-inner js-file-line"><span class="pl-c1">uint16_t</span> <span class="pl-en">MPU6050::dmpGetFIFOPacketSize</span>() {</td>
      </tr>
      <tr>
        <td id="L706" class="blob-num js-line-number" data-line-number="706"></td>
        <td id="LC706" class="blob-code blob-code-inner js-file-line">    <span class="pl-k">return</span> dmpPacketSize;</td>
      </tr>
      <tr>
        <td id="L707" class="blob-num js-line-number" data-line-number="707"></td>
        <td id="LC707" class="blob-code blob-code-inner js-file-line">}</td>
      </tr>
      <tr>
        <td id="L708" class="blob-num js-line-number" data-line-number="708"></td>
        <td id="LC708" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L709" class="blob-num js-line-number" data-line-number="709"></td>
        <td id="LC709" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">endif</span> <span class="pl-c"><span class="pl-c">/*</span> _MPU6050_6AXIS_MOTIONAPPS20_H_ <span class="pl-c">*/</span></span></td>
      </tr>
</table>

  <details class="details-reset details-overlay BlobToolbar position-absolute js-file-line-actions dropdown d-none" aria-hidden="true">
    <summary class="btn-octicon ml-0 px-2 p-0 bg-white border border-gray-dark rounded-1" aria-label="Inline file action toolbar">
      <svg class="octicon octicon-kebab-horizontal" viewBox="0 0 13 16" version="1.1" width="13" height="16" aria-hidden="true"><path fill-rule="evenodd" d="M1.5 9a1.5 1.5 0 1 0 0-3 1.5 1.5 0 0 0 0 3zm5 0a1.5 1.5 0 1 0 0-3 1.5 1.5 0 0 0 0 3zM13 7.5a1.5 1.5 0 1 1-3 0 1.5 1.5 0 0 1 3 0z"/></svg>
    </summary>
    <details-menu>
      <ul class="BlobToolbar-dropdown dropdown-menu dropdown-menu-se mt-2" style="width:185px">
        <li><clipboard-copy role="menuitem" class="dropdown-item" id="js-copy-lines" style="cursor:pointer;" data-original-text="Copy lines">Copy lines</clipboard-copy></li>
        <li><clipboard-copy role="menuitem" class="dropdown-item" id="js-copy-permalink" style="cursor:pointer;" data-original-text="Copy permalink">Copy permalink</clipboard-copy></li>
        <li><a class="dropdown-item js-update-url-with-hash" id="js-view-git-blame" role="menuitem" href="/richardghirst/PiBits/blame/4309bab2cae48600e755452a15c89a7ab3f03dfb/MPU6050-Pi-Demo/MPU6050_6Axis_MotionApps20.h">View git blame</a></li>
          <li><a class="dropdown-item" id="js-new-issue" role="menuitem" href="/richardghirst/PiBits/issues/new">Reference in new issue</a></li>
      </ul>
    </details-menu>
  </details>

  </div>

    </div>

  

  <details class="details-reset details-overlay details-overlay-dark">
    <summary data-hotkey="l" aria-label="Jump to line"></summary>
    <details-dialog class="Box Box--overlay d-flex flex-column anim-fade-in fast linejump" aria-label="Jump to line">
      <!-- '"` --><!-- </textarea></xmp> --></option></form><form class="js-jump-to-line-form Box-body d-flex" action="" accept-charset="UTF-8" method="get"><input name="utf8" type="hidden" value="&#x2713;" />
        <input class="form-control flex-auto mr-3 linejump-input js-jump-to-line-field" type="text" placeholder="Jump to line&hellip;" aria-label="Jump to line" autofocus>
        <button type="submit" class="btn" data-close-dialog>Go</button>
</form>    </details-dialog>
  </details>



  </div>
  <div class="modal-backdrop js-touch-events"></div>
</div>

    </div>
  </div>
  

  </div>

        
<div class="footer container-lg px-3" role="contentinfo">
  <div class="position-relative d-flex flex-justify-between pt-6 pb-2 mt-6 f6 text-gray border-top border-gray-light ">
    <ul class="list-style-none d-flex flex-wrap ">
      <li class="mr-3">&copy; 2019 <span title="0.19027s from unicorn-78b4b88d6-r4nmh">GitHub</span>, Inc.</li>
        <li class="mr-3"><a data-ga-click="Footer, go to terms, text:terms" href="https://github.com/site/terms">Terms</a></li>
        <li class="mr-3"><a data-ga-click="Footer, go to privacy, text:privacy" href="https://github.com/site/privacy">Privacy</a></li>
        <li class="mr-3"><a data-ga-click="Footer, go to security, text:security" href="https://github.com/security">Security</a></li>
        <li class="mr-3"><a href="https://githubstatus.com/" data-ga-click="Footer, go to status, text:status">Status</a></li>
        <li><a data-ga-click="Footer, go to help, text:help" href="https://help.github.com">Help</a></li>
    </ul>

    <a aria-label="Homepage" title="GitHub" class="footer-octicon mr-lg-4" href="https://github.com">
      <svg height="24" class="octicon octicon-mark-github" viewBox="0 0 16 16" version="1.1" width="24" aria-hidden="true"><path fill-rule="evenodd" d="M8 0C3.58 0 0 3.58 0 8c0 3.54 2.29 6.53 5.47 7.59.4.07.55-.17.55-.38 0-.19-.01-.82-.01-1.49-2.01.37-2.53-.49-2.69-.94-.09-.23-.48-.94-.82-1.13-.28-.15-.68-.52-.01-.53.63-.01 1.08.58 1.23.82.72 1.21 1.87.87 2.33.66.07-.52.28-.87.51-1.07-1.78-.2-3.64-.89-3.64-3.95 0-.87.31-1.59.82-2.15-.08-.2-.36-1.02.08-2.12 0 0 .67-.21 2.2.82.64-.18 1.32-.27 2-.27.68 0 1.36.09 2 .27 1.53-1.04 2.2-.82 2.2-.82.44 1.1.16 1.92.08 2.12.51.56.82 1.27.82 2.15 0 3.07-1.87 3.75-3.65 3.95.29.25.54.73.54 1.48 0 1.07-.01 1.93-.01 2.2 0 .21.15.46.55.38A8.013 8.013 0 0 0 16 8c0-4.42-3.58-8-8-8z"/></svg>
</a>
   <ul class="list-style-none d-flex flex-wrap ">
        <li class="mr-3"><a data-ga-click="Footer, go to contact, text:contact" href="https://github.com/contact">Contact GitHub</a></li>
        <li class="mr-3"><a href="https://github.com/pricing" data-ga-click="Footer, go to Pricing, text:Pricing">Pricing</a></li>
      <li class="mr-3"><a href="https://developer.github.com" data-ga-click="Footer, go to api, text:api">API</a></li>
      <li class="mr-3"><a href="https://training.github.com" data-ga-click="Footer, go to training, text:training">Training</a></li>
        <li class="mr-3"><a href="https://github.blog" data-ga-click="Footer, go to blog, text:blog">Blog</a></li>
        <li><a data-ga-click="Footer, go to about, text:about" href="https://github.com/about">About</a></li>

    </ul>
  </div>
  <div class="d-flex flex-justify-center pb-6">
    <span class="f6 text-gray-light"></span>
  </div>
</div>



  <div id="ajax-error-message" class="ajax-error-message flash flash-error">
    <svg class="octicon octicon-alert" viewBox="0 0 16 16" version="1.1" width="16" height="16" aria-hidden="true"><path fill-rule="evenodd" d="M8.893 1.5c-.183-.31-.52-.5-.887-.5s-.703.19-.886.5L.138 13.499a.98.98 0 0 0 0 1.001c.193.31.53.501.886.501h13.964c.367 0 .704-.19.877-.5a1.03 1.03 0 0 0 .01-1.002L8.893 1.5zm.133 11.497H6.987v-2.003h2.039v2.003zm0-3.004H6.987V5.987h2.039v4.006z"/></svg>
    <button type="button" class="flash-close js-ajax-error-dismiss" aria-label="Dismiss error">
      <svg class="octicon octicon-x" viewBox="0 0 12 16" version="1.1" width="12" height="16" aria-hidden="true"><path fill-rule="evenodd" d="M7.48 8l3.75 3.75-1.48 1.48L6 9.48l-3.75 3.75-1.48-1.48L4.52 8 .77 4.25l1.48-1.48L6 6.52l3.75-3.75 1.48 1.48L7.48 8z"/></svg>
    </button>
    You can’t perform that action at this time.
  </div>


    <script crossorigin="anonymous" integrity="sha512-47ZXWJASen/yLysPiEpAb7/WvHBIHIRKo+7W5g0YYFiInTWqpjYBHTpeHut0QWEf51gExNhSEy55XQmxrZ+0xA==" type="application/javascript" src="https://github.githubassets.com/assets/compat-742699bf681282d2e3cf809d2b9de73a.js"></script>
    <script crossorigin="anonymous" integrity="sha512-N6BPdqxnrYL4kxWa5gDIlmhui/SEMiHoobwzTpVOWheR111Zxv5GOnCtGpt5qhE5rIpi9RHMeyngI5w6WhGfnw==" type="application/javascript" src="https://github.githubassets.com/assets/frameworks-0339542411b5666802ea364ae561d67e.js"></script>
    
    <script crossorigin="anonymous" async="async" integrity="sha512-D/8iR8ROD3vVOmwLSVsS1j1knDeAOuW9NLNRFb3Pyd68G/gC1b3xRH/krz0K2nuECEZRjVsUAU5caoJKAwoLwA==" type="application/javascript" src="https://github.githubassets.com/assets/github-27e2e2875f3fc6cfce6518e479adf7b8.js"></script>
    
    
    
  <div class="js-stale-session-flash stale-session-flash flash flash-warn flash-banner d-none">
    <svg class="octicon octicon-alert" viewBox="0 0 16 16" version="1.1" width="16" height="16" aria-hidden="true"><path fill-rule="evenodd" d="M8.893 1.5c-.183-.31-.52-.5-.887-.5s-.703.19-.886.5L.138 13.499a.98.98 0 0 0 0 1.001c.193.31.53.501.886.501h13.964c.367 0 .704-.19.877-.5a1.03 1.03 0 0 0 .01-1.002L8.893 1.5zm.133 11.497H6.987v-2.003h2.039v2.003zm0-3.004H6.987V5.987h2.039v4.006z"/></svg>
    <span class="signed-in-tab-flash">You signed in with another tab or window. <a href="">Reload</a> to refresh your session.</span>
    <span class="signed-out-tab-flash">You signed out in another tab or window. <a href="">Reload</a> to refresh your session.</span>
  </div>
  <template id="site-details-dialog">
  <details class="details-reset details-overlay details-overlay-dark lh-default text-gray-dark" open>
    <summary aria-haspopup="dialog" aria-label="Close dialog"></summary>
    <details-dialog class="Box Box--overlay d-flex flex-column anim-fade-in fast">
      <button class="Box-btn-octicon m-0 btn-octicon position-absolute right-0 top-0" type="button" aria-label="Close dialog" data-close-dialog>
        <svg class="octicon octicon-x" viewBox="0 0 12 16" version="1.1" width="12" height="16" aria-hidden="true"><path fill-rule="evenodd" d="M7.48 8l3.75 3.75-1.48 1.48L6 9.48l-3.75 3.75-1.48-1.48L4.52 8 .77 4.25l1.48-1.48L6 6.52l3.75-3.75 1.48 1.48L7.48 8z"/></svg>
      </button>
      <div class="octocat-spinner my-6 js-details-dialog-spinner"></div>
    </details-dialog>
  </details>
</template>

  <div class="Popover js-hovercard-content position-absolute" style="display: none; outline: none;" tabindex="0">
  <div class="Popover-message Popover-message--bottom-left Popover-message--large Box box-shadow-large" style="width:360px;">
  </div>
</div>

<div id="hovercard-aria-description" class="sr-only">
  Press h to open a hovercard with more details.
</div>

  <div aria-live="polite" class="js-global-screen-reader-notice sr-only"></div>

  </body>
</html>

