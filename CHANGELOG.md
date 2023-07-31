## [Azure Cognitive Search - OpenAI API] Changelog

<a name="Successfully replacing Azure OpenAI with OpenAI API"></a>
# Successfully replacing Azure OpenAI with OpenAI API (2023-07-31)

*Breaking Changes*
* Adapt OpenAI API into approaches
* To change the chatgpt and embedding model, set two azd environment variables OPENAI_GPT_MODEL and OPENAI_EMBED_MODEL, respectively

*What's next*
* Improve the bot's answer

<a name="Modifying the infrastructure"></a>
# Modifying the infrastructure (2023-07-28)

*Breaking Changes*
* Get rid of Azure OpenAI service
* Need to add an azd environment named OPENAI_API_KEY with the OpenAI API Key as value

*What's next*
* Fix bug calling OpenAI Embedding API in `prepdocs.py`