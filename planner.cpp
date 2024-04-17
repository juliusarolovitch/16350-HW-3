#include <iostream>
#include <fstream>
#include <chrono>
#include <memory>
#include <regex>
#include <queue>
#include <unordered_set>
#include <set>
#include <list>
#include <unordered_map>
#include <algorithm>
#include <stdexcept>
#include <stack>
#include <functional>

#define SYMBOLS 0
#define INITIAL 1
#define GOAL 2
#define ACTIONS 3
#define ACTION_DEFINITION 4
#define ACTION_PRECONDITION 5
#define ACTION_EFFECT 6

class GroundedCondition;
class Condition;
class GroundedAction;
class Action;
class Env;

using namespace std;

bool print_status = true;

class GroundedCondition
{
private:
    string predicate;
    list<string> arg_values;
    bool truth = true;

public:
    GroundedCondition(string predicate, list<string> arg_values, bool truth = true)
    {
        this->predicate = predicate;
        this->truth = truth; // fixed
        for (string l : arg_values)
        {
            this->arg_values.push_back(l);
        }
    }

    GroundedCondition(const GroundedCondition &gc)
    {
        this->predicate = gc.predicate;
        this->truth = gc.truth; // fixed
        for (string l : gc.arg_values)
        {
            this->arg_values.push_back(l);
        }
    }

    string get_predicate() const
    {
        return this->predicate;
    }
    list<string> get_arg_values() const
    {
        return this->arg_values;
    }

    bool get_truth() const
    {
        return this->truth;
    }

    friend ostream &operator<<(ostream &os, const GroundedCondition &pred)
    {
        os << pred.toString() << " ";
        return os;
    }

    bool operator==(const GroundedCondition &rhs) const
    {
        if (this->predicate != rhs.predicate || this->arg_values.size() != rhs.arg_values.size())
            return false;

        auto lhs_it = this->arg_values.begin();
        auto rhs_it = rhs.arg_values.begin();

        while (lhs_it != this->arg_values.end() && rhs_it != rhs.arg_values.end())
        {
            if (*lhs_it != *rhs_it)
                return false;
            ++lhs_it;
            ++rhs_it;
        }

        if (this->truth != rhs.get_truth()) // fixed
            return false;

        return true;
    }

    string toString() const
    {
        string temp = "";
        temp += this->predicate;
        temp += "(";
        for (string l : this->arg_values)
        {
            temp += l + ",";
        }
        temp = temp.substr(0, temp.length() - 1);
        temp += ")";
        return temp;
    }
};

struct GroundedConditionComparator
{
    bool operator()(const GroundedCondition &lhs, const GroundedCondition &rhs) const
    {
        return lhs == rhs;
    }
};

struct GroundedConditionHasher
{
    size_t operator()(const GroundedCondition &gcond) const
    {
        return hash<string>{}(gcond.toString());
    }
};

class Condition
{
private:
    string predicate;
    list<string> args;
    bool truth;

public:
    Condition(string pred, list<string> args, bool truth)
    {
        this->predicate = pred;
        this->truth = truth;
        for (string ar : args)
        {
            this->args.push_back(ar);
        }
    }

    string get_predicate() const
    {
        return this->predicate;
    }

    list<string> get_args() const
    {
        return this->args;
    }

    bool get_truth() const
    {
        return this->truth;
    }

    friend ostream &operator<<(ostream &os, const Condition &cond)
    {
        os << cond.toString() << " ";
        return os;
    }

    bool operator==(const Condition &rhs) const // fixed
    {

        if (this->predicate != rhs.predicate || this->args.size() != rhs.args.size())
            return false;

        auto lhs_it = this->args.begin();
        auto rhs_it = rhs.args.begin();

        while (lhs_it != this->args.end() && rhs_it != rhs.args.end())
        {
            if (*lhs_it != *rhs_it)
                return false;
            ++lhs_it;
            ++rhs_it;
        }

        if (this->truth != rhs.get_truth())
            return false;

        return true;
    }

    string toString() const
    {
        string temp = "";
        if (!this->truth)
            temp += "!";
        temp += this->predicate;
        temp += "(";
        for (string l : this->args)
        {
            temp += l + ",";
        }
        temp = temp.substr(0, temp.length() - 1);
        temp += ")";
        return temp;
    }
};

struct ConditionComparator
{
    bool operator()(const Condition &lhs, const Condition &rhs) const
    {
        return lhs == rhs;
    }
};

struct ConditionHasher
{
    size_t operator()(const Condition &cond) const
    {
        return hash<string>{}(cond.toString());
    }
};

class Action
{
private:
    string name;
    list<string> args;
    unordered_set<Condition, ConditionHasher, ConditionComparator> preconditions;
    unordered_set<Condition, ConditionHasher, ConditionComparator> effects;

public:
    Action(string name, list<string> args,
           unordered_set<Condition, ConditionHasher, ConditionComparator> &preconditions,
           unordered_set<Condition, ConditionHasher, ConditionComparator> &effects)
    {
        this->name = name;
        for (string l : args)
        {
            this->args.push_back(l);
        }
        for (Condition pc : preconditions)
        {
            this->preconditions.insert(pc);
        }
        for (Condition pc : effects)
        {
            this->effects.insert(pc);
        }
    }
    string get_name() const
    {
        return this->name;
    }
    list<string> get_args() const
    {
        return this->args;
    }
    unordered_set<Condition, ConditionHasher, ConditionComparator> get_preconditions() const
    {
        return this->preconditions;
    }
    unordered_set<Condition, ConditionHasher, ConditionComparator> get_effects() const
    {
        return this->effects;
    }

    bool operator==(const Action &rhs) const
    {
        if (this->get_name() != rhs.get_name() || this->get_args().size() != rhs.get_args().size())
            return false;

        return true;
    }

    friend ostream &operator<<(ostream &os, const Action &ac)
    {
        os << ac.toString() << endl;
        os << "Precondition: ";
        for (Condition precond : ac.get_preconditions())
            os << precond;
        os << endl;
        os << "Effect: ";
        for (Condition effect : ac.get_effects())
            os << effect;
        os << endl;
        return os;
    }

    string toString() const
    {
        string temp = "";
        temp += this->get_name();
        temp += "(";
        for (string l : this->get_args())
        {
            temp += l + ",";
        }
        temp = temp.substr(0, temp.length() - 1);
        temp += ")";
        return temp;
    }
};

struct ActionComparator
{
    bool operator()(const Action &lhs, const Action &rhs) const
    {
        return lhs == rhs;
    }
};

struct ActionHasher
{
    size_t operator()(const Action &ac) const
    {
        return hash<string>{}(ac.get_name());
    }
};

class Env
{
private:
    unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> initial_conditions;
    unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> goal_conditions;
    unordered_set<Action, ActionHasher, ActionComparator> actions;
    unordered_set<string> symbols;

public:
    void remove_initial_condition(GroundedCondition gc)
    {
        this->initial_conditions.erase(gc);
    }
    void add_initial_condition(GroundedCondition gc)
    {
        this->initial_conditions.insert(gc);
    }
    void add_goal_condition(GroundedCondition gc)
    {
        this->goal_conditions.insert(gc);
    }
    void remove_goal_condition(GroundedCondition gc)
    {
        this->goal_conditions.erase(gc);
    }
    void add_symbol(string symbol)
    {
        symbols.insert(symbol);
    }
    void add_symbols(list<string> symbols)
    {
        for (string l : symbols)
            this->symbols.insert(l);
    }
    void add_action(Action action)
    {
        this->actions.insert(action);
    }

    Action get_action(string name)
    {
        for (Action a : this->actions)
        {
            if (a.get_name() == name)
                return a;
        }
        throw runtime_error("Action " + name + " not found!");
    }
    unordered_set<string> get_symbols() const
    {
        return this->symbols;
    }

    unordered_set<Action, ActionHasher, ActionComparator> get_actions()
    {
        return this->actions;
    }

    unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> get_init_cond() const
    {
        return this->initial_conditions;
    }

    unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> get_goal_cond() const
    {
        return this->goal_conditions;
    }

    friend ostream &operator<<(ostream &os, const Env &w)
    {
        os << "***** Environment *****" << endl
           << endl;
        os << "Symbols: ";
        for (string s : w.get_symbols())
            os << s + ",";
        os << endl;
        os << "Initial conditions: ";
        for (GroundedCondition s : w.initial_conditions)
            os << s;
        os << endl;
        os << "Goal conditions: ";
        for (GroundedCondition g : w.goal_conditions)
            os << g;
        os << endl;
        os << "Actions:" << endl;
        for (Action g : w.actions)
            os << g << endl;
        cout << "***** Environment Created! *****" << endl;
        return os;
    }
};

class GroundedAction
{
private:
    string name;
    list<string> arg_values;

public:
    GroundedAction()
    {
    }
    GroundedAction(string name, list<string> arg_values)
    {
        this->name = name;
        for (string ar : arg_values)
        {
            this->arg_values.push_back(ar);
        }
    }

    string get_name() const
    {
        return this->name;
    }

    list<string> get_arg_values() const
    {
        return this->arg_values;
    }

    bool operator==(const GroundedAction &rhs) const
    {
        if (this->name != rhs.name || this->arg_values.size() != rhs.arg_values.size())
            return false;

        auto lhs_it = this->arg_values.begin();
        auto rhs_it = rhs.arg_values.begin();

        while (lhs_it != this->arg_values.end() && rhs_it != rhs.arg_values.end())
        {
            if (*lhs_it != *rhs_it)
                return false;
            ++lhs_it;
            ++rhs_it;
        }
        return true;
    }

    friend ostream &operator<<(ostream &os, const GroundedAction &gac)
    {
        os << gac.toString() << " ";
        return os;
    }

    string toString() const
    {
        string temp = "";
        temp += this->name;
        temp += "(";
        for (string l : this->arg_values)
        {
            temp += l + ",";
        }
        temp = temp.substr(0, temp.length() - 1);
        temp += ")";
        return temp;
    }
};

list<string> parse_symbols(string symbols_str)
{
    list<string> symbols;
    size_t pos = 0;
    string delimiter = ",";
    while ((pos = symbols_str.find(delimiter)) != string::npos)
    {
        string symbol = symbols_str.substr(0, pos);
        symbols_str.erase(0, pos + delimiter.length());
        symbols.push_back(symbol);
    }
    symbols.push_back(symbols_str);
    return symbols;
}

Env *create_env(char *filename)
{
    ifstream input_file(filename);
    Env *env = new Env();
    regex symbolStateRegex("symbols:", regex::icase);
    regex symbolRegex("([a-zA-Z0-9_, ]+) *");
    regex initialConditionRegex("initialconditions:(.*)", regex::icase);
    regex conditionRegex("(!?[A-Z][a-zA-Z_]*) *\\( *([a-zA-Z0-9_, ]+) *\\)");
    regex goalConditionRegex("goalconditions:(.*)", regex::icase);
    regex actionRegex("actions:", regex::icase);
    regex precondRegex("preconditions:(.*)", regex::icase);
    regex effectRegex("effects:(.*)", regex::icase);
    int parser = SYMBOLS;

    unordered_set<Condition, ConditionHasher, ConditionComparator> preconditions;
    unordered_set<Condition, ConditionHasher, ConditionComparator> effects;
    string action_name;
    string action_args;

    string line;
    if (input_file.is_open())
    {
        while (getline(input_file, line))
        {
            string::iterator end_pos = remove(line.begin(), line.end(), ' ');
            line.erase(end_pos, line.end());

            if (line == "")
                continue;

            if (parser == SYMBOLS)
            {
                smatch results;
                if (regex_search(line, results, symbolStateRegex))
                {
                    line = line.substr(8);
                    sregex_token_iterator iter(line.begin(), line.end(), symbolRegex, 0);
                    sregex_token_iterator end;

                    env->add_symbols(parse_symbols(iter->str())); // fixed

                    parser = INITIAL;
                }
                else
                {
                    cout << "Symbols are not specified correctly." << endl;
                    throw;
                }
            }
            else if (parser == INITIAL)
            {
                const char *line_c = line.c_str();
                if (regex_match(line_c, initialConditionRegex))
                {
                    const std::vector<int> submatches = {1, 2};
                    sregex_token_iterator iter(
                        line.begin(), line.end(), conditionRegex, submatches);
                    sregex_token_iterator end;

                    while (iter != end)
                    {
                        // name
                        string predicate = iter->str();
                        iter++;
                        // args
                        string args = iter->str();
                        iter++;

                        if (predicate[0] == '!')
                        {
                            env->remove_initial_condition(
                                GroundedCondition(predicate.substr(1), parse_symbols(args)));
                        }
                        else
                        {
                            env->add_initial_condition(
                                GroundedCondition(predicate, parse_symbols(args)));
                        }
                    }

                    parser = GOAL;
                }
                else
                {
                    cout << "Initial conditions not specified correctly." << endl;
                    throw;
                }
            }
            else if (parser == GOAL)
            {
                const char *line_c = line.c_str();
                if (regex_match(line_c, goalConditionRegex))
                {
                    const std::vector<int> submatches = {1, 2};
                    sregex_token_iterator iter(
                        line.begin(), line.end(), conditionRegex, submatches);
                    sregex_token_iterator end;

                    while (iter != end)
                    {
                        // name
                        string predicate = iter->str();
                        iter++;
                        // args
                        string args = iter->str();
                        iter++;

                        if (predicate[0] == '!')
                        {
                            env->remove_goal_condition(
                                GroundedCondition(predicate.substr(1), parse_symbols(args)));
                        }
                        else
                        {
                            env->add_goal_condition(
                                GroundedCondition(predicate, parse_symbols(args)));
                        }
                    }

                    parser = ACTIONS;
                }
                else
                {
                    cout << "Goal conditions not specified correctly." << endl;
                    throw;
                }
            }
            else if (parser == ACTIONS)
            {
                const char *line_c = line.c_str();
                if (regex_match(line_c, actionRegex))
                {
                    parser = ACTION_DEFINITION;
                }
                else
                {
                    cout << "Actions not specified correctly." << endl;
                    throw;
                }
            }
            else if (parser == ACTION_DEFINITION)
            {
                const char *line_c = line.c_str();
                if (regex_match(line_c, conditionRegex))
                {
                    const std::vector<int> submatches = {1, 2};
                    sregex_token_iterator iter(
                        line.begin(), line.end(), conditionRegex, submatches);
                    sregex_token_iterator end;
                    // name
                    action_name = iter->str();
                    iter++;
                    // args
                    action_args = iter->str();
                    iter++;

                    parser = ACTION_PRECONDITION;
                }
                else
                {
                    cout << "Action not specified correctly." << endl;
                    throw;
                }
            }
            else if (parser == ACTION_PRECONDITION)
            {
                const char *line_c = line.c_str();
                if (regex_match(line_c, precondRegex))
                {
                    const std::vector<int> submatches = {1, 2};
                    sregex_token_iterator iter(
                        line.begin(), line.end(), conditionRegex, submatches);
                    sregex_token_iterator end;

                    while (iter != end)
                    {
                        // name
                        string predicate = iter->str();
                        iter++;
                        // args
                        string args = iter->str();
                        iter++;

                        bool truth;

                        if (predicate[0] == '!')
                        {
                            predicate = predicate.substr(1);
                            truth = false;
                        }
                        else
                        {
                            truth = true;
                        }

                        Condition precond(predicate, parse_symbols(args), truth);
                        preconditions.insert(precond);
                    }

                    parser = ACTION_EFFECT;
                }
                else
                {
                    cout << "Precondition not specified correctly." << endl;
                    throw;
                }
            }
            else if (parser == ACTION_EFFECT)
            {
                const char *line_c = line.c_str();
                if (regex_match(line_c, effectRegex))
                {
                    const std::vector<int> submatches = {1, 2};
                    sregex_token_iterator iter(
                        line.begin(), line.end(), conditionRegex, submatches);
                    sregex_token_iterator end;

                    while (iter != end)
                    {
                        // name
                        string predicate = iter->str();
                        iter++;
                        // args
                        string args = iter->str();
                        iter++;

                        bool truth;

                        if (predicate[0] == '!')
                        {
                            predicate = predicate.substr(1);
                            truth = false;
                        }
                        else
                        {
                            truth = true;
                        }

                        Condition effect(predicate, parse_symbols(args), truth);
                        effects.insert(effect);
                    }

                    env->add_action(
                        Action(action_name, parse_symbols(action_args), preconditions, effects));

                    preconditions.clear();
                    effects.clear();
                    parser = ACTION_DEFINITION;
                }
                else
                {
                    cout << "Effects not specified correctly." << endl;
                    throw;
                }
            }
        }
        input_file.close();
    }

    else
        cout << "Unable to open file";

    return env;
}

struct GCComparator
{
    bool operator()(const GroundedCondition &lhs, const GroundedCondition &rhs) const
    {
        return lhs.toString() < rhs.toString();
    }
};

struct Node
{
    int g;
    int h;
    shared_ptr<Node> prev;
    GroundedAction prev_action;
    set<GroundedCondition, GCComparator> conditions;

    Node(set<GroundedCondition, GCComparator> &c, int h, int g, shared_ptr<Node> ptr) : conditions(c), g(g), h(h), prev(ptr) {}
};

struct NodeHasher
{
    std::size_t operator()(const std::set<GroundedCondition, GCComparator> &gc_set) const
    {
        std::size_t combined_hash = 0;

        for (const auto &cond : gc_set)
        {
            std::size_t cond_hash = std::hash<std::string>{}(cond.toString());
            combined_hash ^= cond_hash + 0x9e3779b9 + (combined_hash << 6) + (combined_hash >> 2);
        }

        return combined_hash;
    }
};

struct PlannerComparator
{
    bool operator()(const set<GroundedCondition, GCComparator> &lhs, const set<GroundedCondition, GCComparator> &rhs) const
    {
        return lhs == rhs;
    }
};

struct PriorityFunction
{
    bool operator()(const shared_ptr<Node> &n1, const shared_ptr<Node> &n2) const
    {
        return (n1->g + n1->h) > (n2->g + n2->h);
    }
};

priority_queue<shared_ptr<Node>, vector<shared_ptr<Node>>, PriorityFunction> open;
unordered_set<set<GroundedCondition, GCComparator>, NodeHasher, PlannerComparator> closed;

int heuristic(
    set<GroundedCondition, GCComparator> &goal_state,
    set<GroundedCondition, GCComparator> &current_state, bool use_heuristic)
{
    int h = 0;
    if (!use_heuristic)
        return h;
    for (GroundedCondition cond : goal_state)
        if (current_state.find(cond) == current_state.end())
            h++;
    return h;
}

struct ActionNode
{
    unordered_map<GroundedCondition, shared_ptr<ActionNode>, GroundedConditionHasher, GroundedConditionComparator> ActionNodeNext;
    vector<pair<unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator>, GroundedAction>> LeafNodeGA;
};

ActionNode grounded_action_tree;

list<GroundedAction> planner(Env *env)
{
    std::cout << "PLANNER CALLED" << endl;
    bool use_heuristic = true;
    chrono::steady_clock::time_point begin = chrono::steady_clock::now();

    unordered_set<Action, ActionHasher, ActionComparator> all_actions = env->get_actions();
    unordered_set<string> all_symbols = env->get_symbols();
    vector<string> sym;
    for (string s : all_symbols)
        sym.push_back(s);

    vector<string> current;
    vector<vector<string>> combinations;
    set<GroundedCondition, GCComparator> preconditions;

    for (auto &action : all_actions)
    {
        combinations.clear();
        int num_args = action.get_args().size();
        int n = sym.size();
        int k = num_args;
        if (!(k <= 0 || k > n))
        {
            vector<int> indices(k);
            for (int i = 0; i < k; ++i)
                indices[i] = i;

            while (1)
            {
                vector<string> current;
                for (int index : indices)
                    current.push_back(sym[index]);
                combinations.push_back(current);

                int i = k - 1;
                while (i >= 0 && indices[i] == i + n - k)
                    i--;

                if (i < 0)
                    break;

                indices[i]++;
                for (int j = i + 1; j < k; ++j)
                    indices[j] = indices[j - 1] + 1;
            }
        }
        unordered_map<string, string> symbol_map;
        unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> effects;

        for (vector<string> symbol_seq : combinations)
        {
            while (1)
            {
                symbol_map.clear();
                effects.clear();
                preconditions.clear();

                int ind = 0;
                list<string> grounded_action_symbols;
                auto args = action.get_args();
                auto arg_it = args.begin();
                for (size_t i = 0; i < args.size() && arg_it != args.end(); ++i, ++arg_it)
                {
                    symbol_map[*arg_it] = symbol_seq[i];
                    grounded_action_symbols.push_back(symbol_seq[i]);
                }

                auto preconditionsList = action.get_preconditions();
                for (auto it = preconditionsList.begin(); it != preconditionsList.end(); ++it)
                {
                    list<string> actual_args;
                    auto preconditionArgs = it->get_args();
                    for (auto argIt = preconditionArgs.begin(); argIt != preconditionArgs.end(); ++argIt)
                    {
                        actual_args.push_back(symbol_map.find(*argIt) != symbol_map.end() ? symbol_map[*argIt] : *argIt);
                    }
                    GroundedCondition gc_precond(it->get_predicate(), actual_args, it->get_truth());
                    preconditions.insert(gc_precond);
                }

                auto effectsList = action.get_effects();
                for (auto it = effectsList.begin(); it != effectsList.end(); ++it)
                {
                    list<string> actual_args;
                    auto effectArgs = it->get_args();
                    for (auto argIt = effectArgs.begin(); argIt != effectArgs.end(); ++argIt)
                    {
                        actual_args.push_back(symbol_map.find(*argIt) != symbol_map.end() ? symbol_map[*argIt] : *argIt);
                    }
                    GroundedCondition gc_eff(it->get_predicate(), actual_args, it->get_truth());
                    effects.insert(gc_eff);
                }

                GroundedAction ga(action.get_name(), grounded_action_symbols);
                ActionNode *tree = &grounded_action_tree;

                auto it = preconditions.begin();
                while (it != preconditions.end())
                {
                    if (tree->ActionNodeNext.find(*it) == tree->ActionNodeNext.end())
                        tree->ActionNodeNext[*it] = make_shared<ActionNode>();
                    tree = tree->ActionNodeNext[*it].get();

                    if (next(it) == preconditions.end())
                    {
                        tree->LeafNodeGA.push_back({effects, ga});
                        break;
                    }
                    it++;
                }

                if (!next_permutation(symbol_seq.begin(), symbol_seq.end()))
                    break;
            }
        }
    }
    set<GroundedCondition, GCComparator> init, goal;
    for (GroundedCondition val : env->get_init_cond())
        init.insert(val);
    for (GroundedCondition val : env->get_goal_cond())
        goal.insert(val);
    shared_ptr<Node> start = make_shared<Node>(init, heuristic(goal, init, use_heuristic), 0, nullptr);
    unordered_map<set<GroundedCondition, GCComparator>, shared_ptr<Node>, NodeHasher, PlannerComparator> nodes_masterlist;

    nodes_masterlist[init] = start;
    open.push(start);

    while (!open.empty())
    {
        shared_ptr<Node> s = open.top();
        open.pop();

        if (closed.find(s->conditions) != closed.end())
            continue;
        closed.insert(s->conditions);

        if (std::all_of(goal.begin(), goal.end(),
                        [&s](const auto &gc)
                        { return s->conditions.find(gc) != s->conditions.end(); }))
        {
            list<GroundedAction> plan;
            while (s)
            {
                plan.push_back(s->prev_action);
                s = s->prev;
            }
            plan.pop_back();
            plan.reverse();
            chrono::steady_clock::time_point end = chrono::steady_clock::now();

            std::cout << "Time: " << chrono::duration_cast<chrono::milliseconds>(end - begin).count() << " ms" << endl;
            std::cout << endl
                      << "States expanded: " << closed.size() << endl;

            return plan;
        }

        unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> effect;
        GroundedAction grounded_action();

        vector<vector<pair<unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator>, GroundedAction>>> possible_actions;
        queue<pair<ActionNode *, set<GroundedCondition, GCComparator>::const_iterator>> queue;
        queue.push({&grounded_action_tree, s->conditions.begin()});

        while (!queue.empty())
        {
            auto [current, condition] = queue.front();
            queue.pop();

            for (; condition != s->conditions.end(); ++condition)
            {
                if (current->ActionNodeNext.find(*condition) != current->ActionNodeNext.end())
                {
                    ActionNode *next_ptr = current->ActionNodeNext[*condition].get();
                    queue.push({next_ptr, next(condition)});
                }
            }

            if (!current->LeafNodeGA.empty())
                possible_actions.push_back(current->LeafNodeGA);
        }

        for (auto action_iter = possible_actions.begin(); action_iter != possible_actions.end(); ++action_iter)
        {
            for (auto pair_iter = action_iter->begin(); pair_iter != action_iter->end(); ++pair_iter)
            {
                const auto &[effect, grounded_action] = *pair_iter;

                set<GroundedCondition, GCComparator> prev_conds(s->conditions);

                for (const auto &e : effect)
                    if (!e.get_truth())
                        prev_conds.erase(e);
                    else
                        prev_conds.insert(e);

                if (closed.find(prev_conds) != closed.end())
                    continue;

                int h = heuristic(goal, prev_conds, use_heuristic);

                if (nodes_masterlist.find(prev_conds) == nodes_masterlist.end())
                {
                    shared_ptr<Node> newNode = make_shared<Node>(prev_conds, h, s->g + 1, nullptr);
                    nodes_masterlist[prev_conds] = newNode;
                    nodes_masterlist[prev_conds]->prev = s;
                    nodes_masterlist[prev_conds]->prev_action = grounded_action;
                    open.push(nodes_masterlist[prev_conds]);
                }
                else if (nodes_masterlist[prev_conds]->g > (s->g + 1))
                {
                    nodes_masterlist[prev_conds]->g = s->g + 1;
                    nodes_masterlist[prev_conds]->prev = s;
                    nodes_masterlist[prev_conds]->prev_action = grounded_action;
                    open.push(nodes_masterlist[prev_conds]);
                }
            }
        }
    }
    chrono::steady_clock::time_point end = chrono::steady_clock::now();
    std::cout << "Time: " << chrono::duration_cast<chrono::milliseconds>(end - begin).count() << " ms" << endl;
    std::cout << endl
              << "States expanded: " << closed.size() << endl;
    std::cout << "NO PATH FOUND" << endl;
    return list<GroundedAction>{};
}

int main(int argc, char *argv[])
{
    // DO NOT CHANGE THIS FUNCTION
    char *filename = (char *)("example.txt");
    if (argc > 1)
        filename = argv[1];

    cout << "Environment: " << filename << endl
         << endl;
    Env *env = create_env(filename);
    if (print_status)
    {
        cout << *env;
    }

    list<GroundedAction> actions = planner(env);

    cout << "\nPlan: " << endl;
    for (GroundedAction gac : actions)
    {
        cout << gac << endl;
    }

    return 0;
}
