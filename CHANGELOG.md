## [Azure Cognitive Search - OpenAI API] Changelog

<a name="1.1.0"></a>
# 1.1.0 (2023-08-26)

*Breaking Changes*
* Add *Markdown heading splitter* which splits the markdown document by headings and adds a summary of previous parts along with the current part in order to provide the split document more context

*What's next*
* Improve code generation control

<a name="1.0.0"></a>
# 1.0.0 (2023-08-16)

*Breaking Changes*
* Add self-refining feature which repeatedly refine the answer until the answer is close to at least one of retrieved documents

*What's next*
* Improve chunking strategy

<a name="0.1.1"></a>
# 0.1.1 (2023-08-04)

*Breaking Changes*
* Beautify code section in chat UI
* Hard-code the link to citation. Please replace `http://localhost:5000/content/` to `/content/` after debugging

*What's next*
* Improve the bot's answer

<a name="0.1.0"></a>
# 0.1.0 (2023-07-31)

*Breaking Changes*
* Adapt OpenAI API into approaches
* To change the chatgpt and embedding model, set two azd environment variables OPENAI_GPT_MODEL and OPENAI_EMBED_MODEL, respectively

*What's next*
* Improve the bot's answer

<a name="0.0.1"></a>
# 0.0.1 (2023-07-28)

*Breaking Changes*
* Get rid of Azure OpenAI service
* Need to add an azd environment named OPENAI_API_KEY with the OpenAI API Key as value

*What's next*
* Fix bug calling OpenAI Embedding API in `prepdocs.py`