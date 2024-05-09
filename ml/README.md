## Running Experiments for Manipulation

### Installation

```
cd ml/
pip install -r requirements
```

### Execution

* Add the JSON file with the tests inside of dev/eval_tests
* Run `python3 evaluate_run --test_file=<filename.json>`
* All the tests results will be generated inside `eval_data`